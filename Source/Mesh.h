#pragma once

#include "General.h"
#include "Geometry.h"

struct Transform;
class ObjectStructure;
class MeshInstance;

class Mesh : public std::enable_shared_from_this<Mesh> {
public:
	void loadFile(std::string filename);
	std::shared_ptr<MeshInstance> createInstance(const Transform &t);
	const spvector<Object>& getTriangles() { return triangles; }
private:
	spvector<Object> triangles;
};

class MeshInstance : public Object {
public:
	MeshInstance(std::shared_ptr<Mesh> mesh, const Transform &t);

	std::shared_ptr<ObjectStructure> buildObjectStructure() const;

	virtual AABB getAABB() { return aabb; }

private:
	spvector<Object> triangles;
	AABB aabb;
};