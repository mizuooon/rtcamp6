#include "ObjectStructure.h"

void BVHIterator::findNextObject() {
	auto pickShallowerNode = [&]() {
		if ( !objStack.empty() ) {
			currentNode = objStack.top();
			objStack.pop();
			return;
		} else {
			if ( auto parent = currentLocalRootNode->parent.lock() ) {
				lastLocalIndex = currentLocalRootNode->localIndex;
				currentLocalRootNode = parent;
				currentNode = parent->children[lastLocalIndex == 0 ? 1 : 0];
				return;
			} else {
				currentNode = nullptr;
				return;
			}
		}
	};

	if ( currentNode->object != nullptr ) {
		pickShallowerNode();
		if ( currentNode == nullptr ) { return; }
	}

	while ( true ) {
		auto intsct = currentNode->aabb.getIntersection( ray );
		if ( intsct && ( !maxT || *intsct < *maxT ) ) {
			if ( currentNode->object != nullptr ) {
				return;
			} else {
				if ( lastLocalIndex == 0 ) {
					objStack.push( currentNode->children[1] );
					currentNode = currentNode->children[0];
				} else {
					objStack.push( currentNode->children[0] );
					currentNode = currentNode->children[1];
				}
			}
		} else {
			pickShallowerNode();
			if ( currentNode == nullptr ) { return; }
		}
	}
}
