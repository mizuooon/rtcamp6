#pragma once

struct Quaternion {

	float x, y, z, w;

	inline Quaternion() : x( 0.0f ), y( 0.0f ), z( 0.0f ), w( 1.0f ) {}
	inline Quaternion( float x, float y, float z, float w ) : x( x ), y( y ), z( z ), w( w ) {}
	inline Quaternion( const Quaternion &q ) : x( q.x ), y( q.y ), z( q.z ), w( q.w ) {}

	inline Quaternion operator*( const Quaternion &q ) const {
		const auto &q1 = *this;
		const auto &q2 = q;
		return Quaternion(
			q1.w * q2.x + q2.w * q1.x + q1.y*q2.z - q1.z*q2.y,
			q1.w * q2.y + q2.w * q1.y + q1.z*q2.x - q1.x*q2.z,
			q1.w * q2.z + q2.w * q1.z + q1.x*q2.y - q1.y*q2.x,
			q1.w * q2.w - ( q1.x * q2.x + q1.y * q2.y + q1.z * q2.z )
		);
	}

	inline Quaternion conjugate() const {
		return Quaternion( -x, -y, -z, w );
	}

	inline float norm() const {
		return sqrtf( x*x + y * y + z * z + w * w );
	}

	inline Vector3 operator*( const Vector3 &v ) const {
		Quaternion q( v.x, v.y, v.z, 0.0f );
		q = ( *this ) * q * this->conjugate();
		return Vector3( q.x, q.y, q.z );
	}

	inline static Quaternion quaternionRotationAxis( const Vector3 &axis, float angle ) {
		float s = sinf( angle / 2 );
		return Quaternion(
			axis.x* s,
			axis.y * s,
			axis.z *s,
			cosf( angle / 2 )
		);
	}
	static Quaternion identity() {
		return Quaternion( 0, 0, 0, 1 );
	}
};

