#include <DNA.h>
#include <Scene.h>

#include <cstdio>
#include <cassert>
#include <string>

bool Scene::read(physx::PxPhysics & physics, physx::PxScene & scene, std::ifstream & ifile) {
	physx::PxVec3 vertex, zDirection;
	unsigned int edge, numBases;
	std::string line;

	while (ifile.good()) {
		std::getline(ifile, line);

		if (sscanf(line.c_str(), "e %u", &edge) == 1) {
			path.push_back(edge - 1);
		} else if (sscanf(line.c_str(), "v %f %f %f", &vertex.x, &vertex.y, &vertex.z) == 3) {
			vertices.push_back(vertex);
		} else if (sscanf(line.c_str(), "h %u %f %f %f %f %f %f", &numBases, &vertex.x, &vertex.y, &vertex.z, &zDirection.x, &zDirection.y, &zDirection.z) == 7) {
			const physx::PxVec3 cross(kPosZAxis.cross(zDirection));
			helices.push_back(Helix(physics, scene, numBases, physx::PxTransform(vertex, physx::PxQuat(signedAngle(kPosZAxis, zDirection, cross), cross.getNormalized()))));
		}
	}

	assert(helices.empty() || helices.size() == path.size() - 1);
	return helices.empty() ? setupHelices(physics, scene) : true;
}

bool Scene::setupHelices(physx::PxPhysics & physics, physx::PxScene & scene) {
	{
		std::vector<unsigned int>::const_iterator it(path.begin());
		std::vector<unsigned int>::const_iterator prev_it(it++);

		for (; it != path.end(); ++it, ++prev_it) {
			const physx::PxVec3 & vertex1(vertices[*prev_it]), &vertex2(vertices[*it]);
			physx::PxVec3 origo((vertex1 + vertex2) / 2), direction(vertex2 - vertex1);
			const physx::PxReal length(direction.normalize());
			const physx::PxVec3 cross(kPosZAxis.cross(direction));
			helices.push_back(Helix(physics, scene, DNA::DistanceToBaseCount(length), physx::PxTransform(origo, physx::PxQuat(signedAngle(kPosZAxis, direction, cross), cross.getNormalized()))));
		}
	}

	// Connect the scaffold.
	{
		std::vector<Helix>::iterator it(helices.begin());
		std::vector<Helix>::iterator prev_it(it++);

		for (; it != helices.end(); ++it, ++prev_it) {
			// TODO:
			//prev_it->attach(physics, *it, Helix::kForwardThreePrime, Helix::kForwardFivePrime);
		}
	}

	// Connect the staples.

	// Sketch: Iterate over the path, look at where the two edges are going, if positive, look for the closest other edge at this vertex with a negative angle, if not do the inverse.

	return true;
}

bool Scene::write(std::ofstream & ofile) const {
	for (const physx::PxVec3 & vertex : vertices)
		ofile << "v " << vertex.x << ' ' << vertex.y << ' ' << vertex.z << std::endl;

	for (unsigned int edge : path)
		ofile << "e " << edge << std::endl;

	for (const Helix & helix : helices) {
		const physx::PxTransform transform(helix.getTransform());
		const physx::PxVec3 direction(transform.q.rotate(kPosZAxis));
		ofile << "h " << helix.getBaseCount() << ' ' << transform.p.x << ' ' << transform.p.y << ' ' << transform.p.z << ' ' << direction.x << ' ' << direction.y << ' ' << direction.z << std::endl;
	}

	return true;
}