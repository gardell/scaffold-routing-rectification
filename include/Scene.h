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

	struct Vertex {
		physx::PxVec3 position;
		std::vector<unsigned int> neighbors; // Vertex indices.

		// TODO: Return the closest edge relative to the given edges.
		unsigned int findOppositeNeighbor(unsigned int previousVertex, unsigned int nextVertex);

		inline Vertex(const physx::PxVec3 & position) : position(position) {}
	};
	std::vector<Vertex> vertices;
	std::vector<unsigned int> path;
	std::vector<Helix> helices;
};

#endif /* _H_SCENE_H_ */
