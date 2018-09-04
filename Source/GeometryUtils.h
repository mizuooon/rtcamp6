#pragma once

struct BasisVector {
	Vector3 e1, e2, e3;

	Vector3 vector( float x, float y, float z ) { return e1 * x + e2 * y + e3 * z; }
};

inline BasisVector genBasisVector( const Vector3& v ) {
	BasisVector basis;
	basis.e1 = v.normalized();
	basis.e2 = cross( v,
					  fabsf( v.x ) < 0.5f ? Vector3( 1, 0, 0 ) :
					  ( fabsf( v.y ) < 0.5f ? Vector3( 0, 1, 0 ) : Vector3( 0, 0, 1 ) ) ).normalize();
	basis.e3 = cross( basis.e1, basis.e2 ).normalize();
	return std::move( basis );
}

inline Vector3 sampleVectorOnHemiSphere( const Vector3& v ) {
	BasisVector basis = genBasisVector( v );
	float phai = asinf( randf( 1.0f ) );
	float theta = randf( 2.0f * PI );
	return basis.vector( sinf( phai ), cosf( phai ) * cosf( theta ), cosf( phai ) * sinf( theta ) );
}

inline Vector3 sampleVectorOnSphere() {
	float phai = asinf( randf( -1.0f, 1.0f ) );
	float theta = randf( 2.0f * PI );
	return Vector3( sinf( phai ), cosf( phai ) * cosf( theta ), cosf( phai ) * sinf( theta ) );
}