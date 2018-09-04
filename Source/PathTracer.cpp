
#include "General.h"
#include "PathTracer.h"
#include "Geometry.h"
#include "Scene.h"
#include "Material.h"
#include "ObjectStructure.h"

float PathTracer::russianRouretteProbability;
float PathTracer::originOffset;

void PathTracer::evalRadiance( std::shared_ptr<Scene> scene, Ray *ray, std::shared_ptr<ObjectStructureIteratorHistory> history ) {

	if ( ray->depth > 1 && randf() > russianRouretteProbability ) {
		ray->radiance = Vector3( 0.0f );
		return;
	}

	auto intersection = scene->getIntersection( *ray, history, &history );

	if ( intersection ) {
		evalRadiance( scene, ray, *intersection, history );
	} else {
		ray->radiance = Vector3( 0.0f );
	}
	if ( ray->depth > 1 ) {
		*ray->radiance /= russianRouretteProbability;
	}
}

Ray PathTracer::generateNextRay( const Ray &prev, const Vector3 &d, const Vector3 &p, const Intersection &intersection ) {
	Ray next;
	next.d = d;
	next.o = p + d * originOffset;
	next.depth = prev.depth + 1;
	next.media = prev.media;

	if ( intersection.object && dot( prev.d, intersection.n ) * dot( next.d, intersection.n ) > 0.0f ) {
		if ( dot( next.d, intersection.n ) < 0.0f ) {
			next.media.push( intersection.material->participatingMedia );
		} else {
			if ( !next.media.empty() ) {
				next.media.pop();
			} else {
				// ‚¨‚©‚µ‚¢
			}
		}
	}

	return std::move( next );
}

void BSDFSamplingPathTracer::evalRadiance( std::shared_ptr<Scene> scene, Ray *ray, const Intersection &intersection, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
	ray->radiance = Vector3( 0.0f );
	auto bsdfSample = intersection.material->sampleRay( *ray, intersection, scene, history );
	Ray out = generateNextRay( *ray, bsdfSample.d, bsdfSample.p, intersection );

	PathTracer::evalRadiance( scene, &out, history );
	*ray->radiance += clampPositive( *out.radiance * bsdfSample.bsdf_cos_divided_p );
	*ray->radiance += intersection.material->getEmission();
}


