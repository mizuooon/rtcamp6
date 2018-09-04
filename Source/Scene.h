#pragma once

#include "General.h"
#include "Geometry.h"
#include "ObjectStructure.h"

class Scene {
public:
	void addObject(std::shared_ptr<Object> obj) {
		objects.push_back(obj);
	}

	std::shared_ptr<ObjectStructure> buildObjectStructure() {
		return objectStructure = std::make_shared<BVH>(objects);
	}

	std::shared_ptr<ObjectStructure> getObjectStructure() const {
		return objectStructure;
	}

	void addExplicitLight(std::shared_ptr<PrimitiveObject> light) {
		explicitLights.push_back(light);
	}

	const spvector<PrimitiveObject>& getExplicitLights() {
		return explicitLights;
	}

	std::optional<Intersection> getIntersection( const Ray &ray, const std::shared_ptr<ObjectStructureIteratorHistory> &history, std::shared_ptr<ObjectStructureIteratorHistory> *newHistory ) const;
private:
	spvector<Object> objects;
	std::shared_ptr<ObjectStructure> objectStructure;
	spvector<PrimitiveObject> explicitLights;
};
