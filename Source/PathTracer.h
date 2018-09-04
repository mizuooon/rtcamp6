#pragma once

class Scene;
struct ObjectStructureIteratorHistory;
class ExplicitLight;
struct Intersection;
struct Ray;

class PathTracer {
public:
	static float russianRouretteProbability;
	static float originOffset;

	virtual void evalRadiance ( std::shared_ptr<Scene> scene, Ray *ray, std::shared_ptr<ObjectStructureIteratorHistory> history = nullptr );

protected:
	Ray generateNextRay(const Ray &ray, const Vector3 &d, const Vector3 &p, const Intersection &intersection);
	virtual void evalRadiance( std::shared_ptr<Scene> scene, Ray *ray, const Intersection &intersection, std::shared_ptr<ObjectStructureIteratorHistory> history = nullptr ) = 0;
};

class BSDFSamplingPathTracer : public PathTracer{
protected:
	virtual void evalRadiance( std::shared_ptr<Scene> scene, Ray *ray, const Intersection &intersection, std::shared_ptr<ObjectStructureIteratorHistory> history = nullptr );
};
