
#include "General.h"
#include "Scene.h"
#include "Material.h"

std::optional<Intersection> Scene::getIntersection( const Ray &ray, const std::shared_ptr<ObjectStructureIteratorHistory> &history, std::shared_ptr<ObjectStructureIteratorHistory> *newHistory ) const {
	auto objs = getObjectStructure();

	std::optional<Intersection> intersection;
	if ( !ray.media.empty() && ray.media.top() != nullptr ) {
		auto m = ray.media.top();
		Intersection tmp;
		tmp.t = m->sampleDistance();
		tmp.i = -ray.d;
		tmp.p = ray.o + tmp.t * ray.d;
		tmp.n = Vector3( 0, 0, 0 );
		tmp.object = nullptr;
		tmp.material = m;
		intersection = std::move( tmp );
	}

	auto it = objs->traverse( ray, history );
	for ( ; !it->end(); it->next() ) {
		auto tmp = ( *( *it ) )->getIntersection( ray );
		if ( tmp
			 && ( !intersection.has_value() || tmp->t < intersection->t ) ) {
			intersection = std::move( tmp );
			it->select( *intersection );
		}
	}
	if ( newHistory != nullptr ) {
		*newHistory = it->getHistory();
	}

	return intersection;
}
