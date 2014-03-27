#include <Definition.h>
#include <DNA.h>
#include <Utility.h>
#include <Scene.h>
#include <SimulatedAnnealing.h>

#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <Physics.h>

/*
 * Increases/decreases the base counts of individual helices with min(minbasecount, basecount + [ -baserange, baserange ]), and evaluates the new energy level of the system by simulated annealing.
 */
template<typename StoreBestFunctorT, typename RunningFunctorT>
void simulated_annealing(scene & mesh, physics & phys, int kmax, float emax, unsigned int minbasecount, int baserange,
		StoreBestFunctorT store_best_functor, RunningFunctorT running_functor) {

	int modifiedHelix, previousBaseCount;
	physics::transform_type previousTransform;

	scene::HelixContainer & helices(mesh.getHelices());
	const scene::HelixContainer::size_type helixCount(helices.size());

	simulated_annealing(mesh,
		[](scene & mesh) { return mesh.getTotalSeparation(); },
		[&helixCount](float k) { return float(std::max(0., (exp(-k) - 1 / M_E) / (1 - 1 / M_E))) * helixCount; },
		[&modifiedHelix, &helices, &helixCount, &previousBaseCount, &previousTransform, &phys, &minbasecount, &baserange, &running_functor](scene & mesh) {
			for (Helix & helix : helices)
				helix.setTransform(helix.getInitialTransform());

			modifiedHelix = rand() % helixCount;
			Helix & helix(helices[modifiedHelix]);
			previousBaseCount = helix.getBaseCount();
			previousTransform = helix.getTransform();

			helix.recreateRigidBody(
				phys, std::max(minbasecount, helix.getInitialBaseCount() + (rand() % 2 * 2 - 1) * (1 + rand() % (baserange))), helix.getInitialTransform());

			while (!mesh.isSleeping() && running_functor()) {
				phys.scene->simulate(1.0f / 60.0f);
				phys.scene->fetchResults(true);
			}
		},
		probability_functor<float, float>(),
		[&modifiedHelix, &helices, &previousBaseCount, &previousTransform, &phys](scene & mesh) {
			Helix & helix(helices[modifiedHelix]);
			helix.recreateRigidBody(phys, previousBaseCount, helix.getInitialTransform());
		},
		store_best_functor,
		running_functor,
		kmax, emax);
}

/*
 * Simple gradient descent implementation: Energy lower? Choose it, if not don't.
 */
template<typename StoreBestFunctorT, typename RunningFunctorT>
void gradient_descent(scene & mesh, physics & phys, int minbasecount, StoreBestFunctorT store_best_functor, RunningFunctorT running_functor) {
	scene::HelixContainer & helices(mesh.getHelices());

	while (!mesh.isSleeping() && running_functor()) {
		phys.scene->simulate(1.0f / 60.0f);
		phys.scene->fetchResults(true);
	}

	//physics::real_type separation(mesh.getTotalSeparation());
	physics::real_type min, max, average, total;
	mesh.getTotalSeparationMinMaxAverage(min, max, average, total);
	store_best_functor(mesh, min, max, average, total);

	for (Helix & helix : helices) {
		for (int i = 0; i < 2; ++i) {
			if (!running_functor())
				return;

			helix.recreateRigidBody(phys, std::max(minbasecount, int(helix.getInitialBaseCount() + (i * 2 - 1))), helix.getInitialTransform());

			while (!mesh.isSleeping() && running_functor()) {
				phys.scene->simulate(1.0f / 60.0f);
				phys.scene->fetchResults(true);
			}

			//const physics::real_type newseparation(mesh.getTotalSeparation());
			physics::real_type newtotal;
			mesh.getTotalSeparationMinMaxAverage(min, max, average, newtotal);

			if (newtotal < total) {
				total = newtotal;
				store_best_functor(mesh, min, max, average, total);
			} else
				helix.recreateRigidBody(phys, helix.getInitialBaseCount(), helix.getInitialTransform());

			for (Helix & helix : helices)
				helix.setTransform(helix.getInitialTransform());
		}
	}
}

volatile bool running = true;

void handle_exit() {
	running = false;
}

/*
 * Does a simple rectification of the structure without modification.
 */
template<typename RunningFunctorT>
SceneDescription simulated_rectification(scene & mesh, physics & phys, RunningFunctorT running_functor) {
	while (!mesh.isSleeping() && running_functor()) {
		phys.scene->simulate(1.0f / 60.0f);
		phys.scene->fetchResults(true);
	}

	return SceneDescription(mesh);
}

int main(int argc, const char **argv) {
	seed();

	if (argc != 3) {
		std::cerr << "Usage: " << argv[0] << " <input file> <output file>" << std::endl << "\tExample: " << argv[0] << " input.rmsh output.vhelix" << std::endl;
		return 0;
	}

	physics::settings_type physics_settings;
	physics_settings.kStaticFriction = physics::real_type(0.5);
	physics_settings.kDynamicFriction = physics::real_type(0.5);
	physics_settings.kRestitution = physics::real_type(1.0);
	physics_settings.kRigidBodySleepThreshold = physics::real_type(0.001);
	physics_settings.visual_debugger = true;
	physics phys(physics_settings);

	scene::settings_type scene_settings;
	scene_settings.initial_scaling = physics::real_type(1.0);
	Helix::settings_type helix_settings;
	helix_settings.attach_fixed = true;
	helix_settings.density = physics::real_type(10);
	helix_settings.spring_stiffness = physics::real_type(100);
	helix_settings.fixed_spring_stiffness = physics::real_type(1000);
	helix_settings.spring_damping = physics::real_type(100);
	scene mesh(scene_settings, helix_settings);

	std::ifstream infile(argv[1]);
	if (!mesh.read(phys, infile)) {
		std::cerr << "Failed to read scene \"" << argv[1] << "\"" << std::endl;
		return 1;
	}
	infile.close();

	//const physics::real_type initialSeparation(mesh.getTotalSeparation());
	physics::real_type initialmin, initialmax, initialaverage, initialtotal, min, max, average, total;
	mesh.getTotalSeparationMinMaxAverage(initialmin, initialmax, initialaverage, initialtotal);

	std::cerr << "Running simulation for scene loaded from \"" << argv[1] << "\"." << std::endl
		<< "Initial: min: " << initialmin << ", max: " << initialmax << ", average: " << initialaverage << ", total: " << initialtotal << " nm" << std::endl
		<< "Connect with NVIDIA PhysX Visual Debugger to " << PVD_HOST << ':' << PVD_PORT << " to visualize the progress. " << std::endl
		<< "Press ^C to stop the relaxation...." << std::endl;

	setinterrupthandler<handle_exit>();

	SceneDescription best_scene;
#if 0
	best_scene = simulated_rectification(mesh, phys, []() { return running; });
#else
#if 0
	simulated_annealing(mesh, phys, 100, 0, 7, 1,
		[&best_scene](scene & mesh, float e) { std::cerr << "Store best energy: " << e << std::endl; best_scene = SceneDescription(mesh); },
		[]() { return running; });
#else
	gradient_descent(mesh, phys, 7,
		[&best_scene, &min, &max, &average, &total](scene & mesh, physics::real_type min_, physics::real_type max_, physics::real_type average_, physics::real_type total_) { min = min_; max = max_; average = average_; total = total_; std::cerr << "State: min: " << min << ", max: " << max << ", average: " << average << " total: " << total << " nm" << std::endl; best_scene = SceneDescription(mesh); },
		[]() { return running; });
#endif
#endif

	std::cerr << "Result: min: " << min << ", max: " << max << ", average: " << average << ", total: " << total << " nm" << std::endl;

	{
		std::ofstream outfile(argv[2]);
		outfile << "# Relaxation of original " << argv[1] << " file. " << mesh.getHelixCount() << " helices." << std::endl
			<< "# Total separation: Initial: min: " << initialmin << ", max: " << initialmax << ", average: " << initialaverage << ", total: " << initialtotal << " nm" << ", final: min: " << min << ", max: " << max << ", average: " << average << ", total: " << total << " nm" << std::endl;

		if (!best_scene.write(outfile))
			std::cerr << "Failed to write resulting mesh to \"" << argv[2] << "\"" << std::endl;

		outfile.close();
	}

	sleepms(2000);

	return 0;
}
