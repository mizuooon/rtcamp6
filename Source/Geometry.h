#pragma once

class ObjectStructure;
struct Ray;
struct Material;
struct Object;
struct PrimitiveObject;
struct ParticipatingMedia;

template <class T,class U>
struct Sample{
	T sample;
	float probablity;
	std::function<U> pdf;
};

struct Transform {
	Vector3 position;
	Vector3 scale;
	Quaternion rotation;

	Transform() : position( 0.0f ), scale( 1.0f ), rotation() {}
	Transform( const Vector3 &position, const Vector3 &scale, const Quaternion &rotation ) :
		position( position ), scale( scale ), rotation( rotation ) {}

};

struct Ray {
	Vector3 o;
	Vector3 d;
	std::optional<Vector3> radiance;
	int depth;

	std::stack<std::shared_ptr<ParticipatingMedia>> media;
};

struct Intersection {
	Vector3 p;
	Vector3 n;
	Vector3 i;
	Vector2 uv;
	float t;
	std::shared_ptr<Material> material;
	std::shared_ptr<const PrimitiveObject> object;
};

struct PointOnSurface {
	Vector3 p;
	Vector3 n;
};

struct Box {

};

struct AABB {
	Vector3 min;
	Vector3 max;

	AABB operator|( const AABB& aabb ) {
		return AABB{ ::min( min, aabb.min ), ::max( max, aabb.max ) };
	}
	AABB& operator|=( const AABB& aabb ) {
		return *this = *this | aabb;
	}

	std::optional<float> getIntersection( const Ray &ray );
};

struct Object {
	virtual AABB getAABB() = 0;

};

struct PrimitiveObject : public Object, public std::enable_shared_from_this<PrimitiveObject> {
	PrimitiveObject( std::shared_ptr<Material> material = nullptr ) : material( material ) {}

	virtual std::optional<Intersection> getIntersection( const Ray &ray ) = 0;

	std::shared_ptr<Material> material;
};

inline AABB getAABB( const spvector<Object>::const_iterator &begin, const spvector<Object>::const_iterator &end ) {
	AABB aabb = ( *begin )->getAABB();
	for ( auto it = begin + 1; it != end; it++ ) {
		aabb |= ( *it )->getAABB();
	}
	return std::move( aabb );
}

inline AABB getAABB( const spvector<Object>& objects ) {
	return getAABB( objects.begin(), objects.end() );
}

inline AABB getAABB( const spvector<PrimitiveObject>::const_iterator &begin, const spvector<PrimitiveObject>::const_iterator &end ) {
	AABB aabb = ( *begin )->getAABB();
	for ( auto it = begin + 1; it != end; it++ ) {
		aabb |= ( *it )->getAABB();
	}
	return std::move( aabb );
}

inline AABB getAABB( const spvector<PrimitiveObject>& objects ) {
	return getAABB( objects.begin(), objects.end() );
}

struct Sphere : public PrimitiveObject {
	Sphere( const Vector3 &center, float radius, std::shared_ptr<Material> material ) : PrimitiveObject( material ), center( center ), radius( radius ) {}
	Vector3 center;
	float radius;
	virtual std::optional<Intersection> getIntersection( const Ray &ray );
	virtual AABB getAABB() {
		return AABB{ center - Vector3( radius ), center + Vector3( radius ) };
	}

	virtual Vector3 getRadiance( const Vector3 &p, const Vector3 &o) const;
};


struct Vertex {
	Vector3 p;
	Vector3 n;
	Vector2 texCoord;
};

struct Triangle : public PrimitiveObject {
	Vertex v[3];
	virtual std::optional<Intersection> getIntersection( const Ray &ray );
	virtual AABB getAABB() { return AABB{ min( v[0].p,v[1].p,v[2].p ),max( v[0].p,v[1].p,v[2].p ) }; }

	void calcNormal() {
		// 頂点位置から Vertex の n を計算
		// メッシュ情報から n を入力しないとき用
		Vector3 n = cross( v[1].p - v[0].p, v[2].p - v[1].p ).normalize();
		v[0].n = n;
		v[1].n = n;
		v[2].n = n;
	}

	virtual Vector3 getRadiance( const Vector3 &p, const Vector3 &o ) const;

};