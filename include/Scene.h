#ifndef _H_SCENE_H_
#define _H_SCENE_H_

#include <Utility.h>

#include <array>
#include <fstream>
#include <vector>

#include <Helix.h>

class Scene {
public:

	bool read(physx::PxPhysics & physics, physx::PxScene & scene, std::ifstream & ifile);
	bool write(std::ofstream & ofile) const;

private:
	bool setupHelices(physx::PxPhysics & physics, physx::PxScene & scene);

	std::vector<physx::PxVec3> vertices;
	std::vector<unsigned int> path;
	std::vector<Helix> helices;
};

#endif /* _H_SCENE_H_ */
