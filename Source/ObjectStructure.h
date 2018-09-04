#pragma once

#include "General.h"
#include "Geometry.h"
#include "Mesh.h"


class ObjectStructure;
class BVH;


struct ObjectStructureIteratorHistory {
};

class ObjectStructureIterator {
public:
	virtual std::shared_ptr<PrimitiveObject> operator*() const = 0;
	virtual ObjectStructureIterator& next() = 0;
	virtual bool end() const = 0;
	virtual void select(const Intersection& intersection) = 0;
	virtual std::shared_ptr<ObjectStructureIteratorHistory> getHistory() = 0;
};
class ObjectStructure {
public:
	virtual std::shared_ptr<ObjectStructureIterator> traverse(const Ray& ray, std::shared_ptr<ObjectStructureIteratorHistory> history) = 0;
};

class NaiveObjectStructureIterator : public ObjectStructureIterator {
public:
	NaiveObjectStructureIterator(spvector<PrimitiveObject> &objects) :
		iterator(objects.begin()), end_iterator(objects.end()) {}
	virtual std::shared_ptr<PrimitiveObject> operator*() const { return *iterator; }
	virtual ObjectStructureIterator& next() { ++iterator; return *this; }
	virtual bool end() const { return iterator == end_iterator; }
	virtual void select(const Intersection& intersection) {}
	virtual std::shared_ptr<ObjectStructureIteratorHistory> getHistory() { return nullptr; }
private:
	spvector<PrimitiveObject>::iterator iterator;
	spvector<PrimitiveObject>::const_iterator end_iterator;
};
class NaiveObjectStructure : public ObjectStructure {
public:
	NaiveObjectStructure(const spvector<PrimitiveObject> &objects) : objects(objects) {}
	virtual std::shared_ptr<ObjectStructureIterator> traverse(const Ray& ray, std::shared_ptr<ObjectStructureIteratorHistory> history) { return std::make_shared<NaiveObjectStructureIterator>(objects); }
private:
	spvector<PrimitiveObject> objects;
};



struct BVHNode {
	AABB aabb;
	std::weak_ptr<BVHNode> parent;
	int localIndex; // 自身の親に対する子の中でのインデックス
	std::shared_ptr<PrimitiveObject> object;
	std::shared_ptr<BVHNode> children[2];
};

struct BVHIteratorHistory : public ObjectStructureIteratorHistory {
	BVHIteratorHistory(std::shared_ptr<BVHNode> lastSelectedNode) : lastSelectedNode(lastSelectedNode) {}
	std::shared_ptr<BVHNode> lastSelectedNode;
};

class BVHIterator : public ObjectStructureIterator {
public:
	BVHIterator(std::shared_ptr<BVH> objectStructure, std::shared_ptr<BVHNode> node, const Ray& ray, std::shared_ptr<ObjectStructureIteratorHistory> history)
		: objectStructure(objectStructure), currentNode(node), ray(ray) {

		if (history != nullptr && std::static_pointer_cast<BVHIteratorHistory>(history)->lastSelectedNode != nullptr) {
			currentLocalRootNode = std::static_pointer_cast<BVHIteratorHistory>(history)->lastSelectedNode;
			currentNode = currentLocalRootNode;
		}
		else {
			currentLocalRootNode = node;
			findNextObject();
		}
		lastLocalIndex = 0;
	}
	virtual std::shared_ptr<PrimitiveObject> operator*() const { return currentNode->object; }
	virtual ObjectStructureIterator& next() { findNextObject(); return *this; }
	virtual bool end() const { return currentNode == nullptr; }
	virtual void select(const Intersection& intersection) {
		selectedNode = currentNode;
		maxT = intersection.t;
	}
	virtual std::shared_ptr<ObjectStructureIteratorHistory> getHistory() { return std::make_shared<BVHIteratorHistory>(selectedNode); }
private:

	void findNextObject();

	std::optional<float> maxT;
	std::shared_ptr<BVHNode> currentNode;
	std::shared_ptr<BVHNode> selectedNode;
	std::stack<std::shared_ptr<BVHNode>> objStack;
	Ray ray;
	std::shared_ptr<BVH> objectStructure;

	int lastLocalIndex;
	std::shared_ptr<BVHNode> currentLocalRootNode;

};

class BVH : public ObjectStructure, public std::enable_shared_from_this<BVH> {
public:
	using AABBObj = std::pair<AABB, std::shared_ptr<Object>>;

	BVH(const spvector<Object> &objects) {
		std::vector<AABBObj> aabbObjects;
		aabbObjects.reserve(objects.size());
		for (auto &obj : objects) { aabbObjects.push_back(std::make_pair(obj->getAABB(), obj)); }
		node = std::make_shared<BVHNode>();
		node->localIndex = 0;
		buildBVH(aabbObjects.begin(), aabbObjects.end(), node);
	}
	virtual std::shared_ptr<ObjectStructureIterator> traverse(const Ray& ray, std::shared_ptr<ObjectStructureIteratorHistory> history) { return std::make_shared<BVHIterator>(shared_from_this(), node, ray, history); }

	std::shared_ptr<BVHNode> getRootNode() { return node; }

private:
	std::shared_ptr<BVHNode> node;

	std::shared_ptr<BVHNode> buildBVH(std::vector<AABBObj>::iterator begin, const std::vector<AABBObj>::iterator &end, const std::shared_ptr<BVHNode> &node) {

		if (end - begin == 1) {
			if (auto mesh = std::dynamic_pointer_cast<MeshInstance>(begin->second)) {
				auto meshBVHNode = std::static_pointer_cast<BVH>(mesh->buildObjectStructure())->getRootNode();

				if (auto parent = node->parent.lock()) {
					parent->children[node->localIndex] = meshBVHNode;
					meshBVHNode->parent = parent;
				}
				return meshBVHNode;
			}
			else {
				node->object = std::static_pointer_cast<PrimitiveObject>(begin->second);
				node->aabb = node->object->getAABB();
			}
			return node;
		}

		int bestIndex;
		{
			int bestAxis = -1;
			float bestSAH;
			AABB aabb1;
			std::stack<AABB> aabb2;
			const int objNum = (int)(end - begin);

			auto getAABB = [](const std::vector<AABBObj>::const_iterator &begin, const std::vector<AABBObj>::const_iterator &end) {
				AABB aabb = begin->first;
				for (auto it = begin; it != end; it++) {
					aabb |= it->first;
				}
				return std::move(aabb);
			};
			const AABB aabb = getAABB(begin, end);
			node->aabb = aabb;

			std::vector<AABBObj> xSortedObjs(begin, end);
			std::sort(xSortedObjs.begin(), xSortedObjs.end(), [](const AABBObj &a, const AABBObj &b) {
				return (a.first.min.x+a.first.max.x) < (b.first.min.x + b.first.max.x);
			});

			std::vector<AABBObj> ySortedObjs(begin, end);
			std::sort(ySortedObjs.begin(), ySortedObjs.end(), [](const AABBObj &a, const AABBObj &b) {
				return (a.first.min.y + a.first.max.y) < (b.first.min.y + b.first.max.y);
			});

			std::vector<AABBObj> zSortedObjs(begin, end);
			std::sort(zSortedObjs.begin(), zSortedObjs.end(), [](const AABBObj &a, const AABBObj &b) {
				return (a.first.min.z + a.first.max.z) < (b.first.min.z + b.first.max.z);
			});

			auto A = [](const AABB& aabb) {
				auto size = aabb.max - aabb.min;
				return 2.0f * (size.x * size.y + size.y * size.z + size.z * size.x);
			};
			auto calcSAH = [&A](const AABB& aabb, const AABB& aabb1, const AABB& aabb2, int objNum1, int objNum2) {
				const float T_AABB = 1.0f;
				const float T_tri = 1.0f;
				//float A_S = A(aabb);
				//return 2 * T_AABB + A(aabb1) / A_S * objNum1 * T_tri + A(aabb2) / A_S * objNum2 * T_tri;
				return A(aabb1) * objNum1 + A(aabb2) * objNum2;
			};

			// TODO : 再帰の最中に何度もソートし直す必要ない

			aabb1 = xSortedObjs[0].second->getAABB();
			aabb2.push(xSortedObjs.rbegin()->first);
			for (auto it = xSortedObjs.rbegin()+1; it != xSortedObjs.rend()-1; it++) { aabb2.push(aabb2.top() | it->first); }
			for (int i = 1; i < xSortedObjs.size(); i++) {
				aabb1 |= xSortedObjs[i].first;
				float sah = calcSAH(aabb, aabb1, aabb2.top(), i, objNum - i);
				if (bestAxis == -1 || sah < bestSAH) {
					bestAxis = 0;
					bestIndex = i;
					bestSAH = sah;
				}
				aabb2.pop();
			}

			aabb1 = ySortedObjs[0].second->getAABB();
			aabb2.push(ySortedObjs.rbegin()->first);
			for (auto it = ySortedObjs.rbegin() + 1; it != ySortedObjs.rend() - 1; it++) { aabb2.push(aabb2.top() | it->first); }
			for (int i = 1; i < ySortedObjs.size(); i++) {
				aabb1 |= ySortedObjs[i].first;
				float sah = calcSAH(aabb, aabb1, aabb2.top(), i, objNum - i);
				if (bestAxis == -1 || sah < bestSAH) {
					bestAxis = 1;
					bestIndex = i;
					bestSAH = sah;
				}
				aabb2.pop();
			}

			aabb1 = zSortedObjs[0].second->getAABB();
			aabb2.push(zSortedObjs.rbegin()->first);
			for (auto it = zSortedObjs.rbegin() + 1; it != zSortedObjs.rend() - 1; it++) { aabb2.push(aabb2.top() | it->first); }
			for (int i = 1; i < zSortedObjs.size(); i++) {
				aabb1 |= zSortedObjs[i].first;
				float sah = calcSAH(aabb, aabb1, aabb2.top(), i, objNum - i);
				if (bestAxis == -1 || sah < bestSAH) {
					bestAxis = 2;
					bestIndex = i;
					bestSAH = sah;
				}
				aabb2.pop();
			}

			auto &bestSortedObjs = bestAxis == 0 ? xSortedObjs : (bestAxis == 1 ? ySortedObjs : zSortedObjs);
			std::copy(bestSortedObjs.begin(), bestSortedObjs.end(), begin);
		}

		node->children[0] = std::make_shared<BVHNode>();
		node->children[1] = std::make_shared<BVHNode>();

		node->children[0]->parent = node;
		node->children[1]->parent = node;
		node->children[0]->localIndex = 0;
		node->children[1]->localIndex = 1;

		buildBVH(begin, begin + bestIndex, node->children[0]);
		buildBVH(begin + bestIndex, end, node->children[1]);

		return node;
	}
};

inline std::shared_ptr<ObjectStructure> buildObjectStructure(const spvector<Object> objects) {
	return std::make_shared<BVH>(objects);
}