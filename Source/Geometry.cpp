#include "General.h"
#include "Geometry.h"
#include "Scene.h"
#include "Material.h"

std::optional<float> AABB::getIntersection( const Ray &ray ) {

	const auto &o = ray.o;
	const auto &d = ray.d;

	if ( inRange( o.x, min.x, max.x ) && inRange( o.y, min.y, max.y ) && inRange( o.z, min.z, max.z ) ) { return true; }

	float x = d.x > 0.0f ? min.x : max.x;
	float y = d.y > 0.0f ? min.y : max.y;
	float z = d.z > 0.0f ? min.z : max.z;

	float validX = fabs( d.x ) > 0.00001f;
	float validY = fabs( d.y ) > 0.00001f;
	float validZ = fabs( d.z ) > 0.00001f;

	float tx = validX ? ( x - o.x ) / d.x : 0.0f;
	float ty = validY ? ( y - o.y ) / d.y : 0.0f;
	float tz = validZ ? ( z - o.z ) / d.z : 0.0f;

	Vector3 px = o + tx * d;
	Vector3 py = o + ty * d;
	Vector3 pz = o + tz * d;

	if ( validX && tx >= 0.0f && inRange( px.y, min.y, max.y ) && inRange( px.z, min.z, max.z ) ) {
		return tx;
	}
	if ( validY && ty >= 0.0f && inRange( py.z, min.z, max.z ) && inRange( py.x, min.x, max.x ) ) {
		return ty;
	}
	if ( validZ && tz >= 0.0f && inRange( pz.x, min.x, max.x ) && inRange( pz.y, min.y, max.y ) ) {
		return tz;
	}

	return std::nullopt;
}


std::optional<Intersection> Sphere::getIntersection( const Ray &ray ) {

	float A = ray.d.lengthSq();
	float B = dot( ray.d, ray.o - center );
	float C = ( ray.o - center ).lengthSq() - radius * radius;

	float discriminant = B * B - A * C;
	if ( discriminant < 0.0f ) {
		return std::nullopt;
	}

	bool back = false;
	float t = ( -B - sqrtf( discriminant ) ) / A;
	if ( t < 0.0f ) {
		t = ( -B + sqrtf( discriminant ) ) / A;
		if ( t < 0.0f ) {
			return std::nullopt;
		}
		back = true;
	}

	Intersection result;
	result.p = ray.o + t * ray.d;
	result.t = t;
	result.n = ( result.p - center ).normalize();
	result.material = material;
	result.i = -ray.d;
	result.object = shared_from_this();

	result.uv = Vector2((result.n.x+1)/2, (result.n.y+1)/2);

	return std::move( result );
}

Vector3 Sphere::getRadiance( const Vector3 &p, const Vector3 &o ) const {
	return material->getEmission();
}

std::optional<Intersection> Triangle::getIntersection( const Ray &ray ) {
	auto &o = ray.o;
	auto &d = ray.d;
	auto &p0 = v[0].p;
	auto &p1 = v[1].p;
	auto &p2 = v[2].p;


	auto v01 = p1 - p0;
	auto v02 = p2 - p0;
	auto v12 = p2 - p1;

	auto n = cross( v01, v02 ).normalize();
	if ( dot( n, d ) > 0.0f ) { n *= -1.0f; }

	float d_dot_n = dot( d, n );
	if ( fabs( d_dot_n ) < 0.0000001f ) { return std::nullopt; }

	float t = dot( p0 - o, n ) / d_dot_n;

	if ( t < 0.0f ) { return std::nullopt; }

	auto p = o + t * d;

	const auto vop0 = p0 - o;
	const auto vop1 = p1 - o;
	const auto vop2 = p2 - o;

	float V0 = dot( p - o, cross( vop1, vop2 ) );
	float V1 = dot( p - o, cross( vop2, vop0 ) );
	float V2 = dot( p - o, cross( vop0, vop1 ) );
	float V = V0 + V1 + V2;
	float a = V0 / V;
	float b = V1 / V;
	float c = V2 / V;

	if ( !inRange01( a ) || !inRange01( b ) || !inRange01( c ) ) { return std::nullopt; }

	Intersection result;
	result.p = std::move( p );
	result.t = t;
	result.n = ( a * v[0].n + b * v[1].n + c * v[2].n ).normalize();
	result.uv = a * v[0].texCoord + b * v[1].texCoord + c * v[2].texCoord;
	result.i = -ray.d;
	result.material = material;
	result.object = shared_from_this();

	return std::move( result );
}

Vector3 Triangle::getRadiance( const Vector3 &p, const Vector3 &o ) const {
	return material->getEmission();
}
