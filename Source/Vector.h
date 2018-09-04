#pragma once

struct Quaternion;


struct Vector2 {
	float x, y;

	inline Vector2() : x( 0.0f ), y( 0.0f ){}
	inline Vector2( float x, float y) : x( x ), y( y ) {}
	inline Vector2( float v ) : x( v ), y( v ) {}
	inline Vector2( const Vector2& v ) : x( v.x ), y( v.y ) {}

	inline Vector2& operator=( const Vector2 &v ) {
		x = v.x;
		y = v.y;
		return *this;
	}
	inline bool operator==( const Vector2 &v ) const { return x == v.x && y == v.y; }
	inline bool operator!=( const Vector2 &v ) const { return !( ( *this ) == v ); }

	inline Vector2 operator+( const Vector2 &v ) const { return Vector2( x + v.x, y + v.y ); }
	inline Vector2 operator-( const Vector2 &v ) const { return Vector2( x - v.x, y - v.y ); }
	inline Vector2 operator*( const Vector2 &v ) const { return Vector2( x * v.x, y * v.y ); }
	inline Vector2 operator/( const Vector2 &v ) const { return Vector2( x / v.x, y / v.y ); }
	inline Vector2 operator-() const { return ( *this ) * ( -1 ); }
	inline Vector2 operator/( float v ) const { return Vector2( x / v, y / v); }
	inline Vector2& operator+=( const Vector2 &v ) {
		x += v.x;
		y += v.y;
		return *this;
	}
	inline Vector2& operator-=( const Vector2 &v ) {
		x -= v.x;
		y -= v.y;
		return *this;
	}
	inline Vector2& operator*=( const Vector2 &v ) {
		x *= v.x;
		y *= v.y;
		return *this;
	}
	inline Vector2& operator/=( const Vector2 &v ) {
		x /= v.x;
		y /= v.y;
		return *this;
	}
	inline Vector2& operator/=( float v ) {
		x /= v;
		y /= v;
		return *this;
	}

	inline float length() const { return sqrtf( x * x + y * y ); }
	inline float lengthSq() const { return x * x + y * y; }

	inline Vector2& normalize() { *this /= length(); return *this; }
	inline Vector2 normalized() const { return *this / length(); }
};

struct Vector3 {
	float x, y, z;

	inline Vector3() : x( 0.0f ), y( 0.0f ), z( 0.0f ) {}
	inline Vector3( float x, float y, float z ) : x( x ), y( y ), z( z ) {}
	inline Vector3( float v ) : x( v ), y( v ), z( v ) {}
	inline Vector3( const Vector3& v ) : x( v.x ), y( v.y ), z( v.z ) {}

	inline Vector3& operator=( const Vector3 &v ) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}
	inline bool operator==( const Vector3 &v ) const { return x == v.x && y == v.y && z == v.z; }
	inline bool operator!=( const Vector3 &v ) const { return !( ( *this ) == v ); }

	inline Vector3 operator+( const Vector3 &v ) const { return Vector3( x + v.x, y + v.y, z + v.z ); }
	inline Vector3 operator-( const Vector3 &v ) const { return Vector3( x - v.x, y - v.y, z - v.z ); }
	inline Vector3 operator*( const Vector3 &v ) const { return Vector3( x * v.x, y * v.y, z * v.z ); }
	inline Vector3 operator/( const Vector3 &v ) const { return Vector3( x / v.x, y / v.y, z / v.z ); }
	inline Vector3 operator-() const { return ( *this ) * ( -1 ); }
	inline Vector3 operator/( float v ) const { return Vector3( x / v, y / v, z / v ); }
	inline Vector3& operator+=( const Vector3 &v ) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	inline Vector3& operator-=( const Vector3 &v ) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}
	inline Vector3& operator*=( const Vector3 &v ) {
		x *= v.x;
		y *= v.y;
		z *= v.z;
		return *this;
	}
	inline Vector3& operator/=( const Vector3 &v ) {
		x /= v.x;
		y /= v.y;
		z /= v.z;
		return *this;
	}
	inline Vector3& operator/=( float v ) {
		x /= v;
		y /= v;
		z /= v;
		return *this;
	}

	inline float length() const { return sqrtf( x * x + y * y + z * z ); }
	inline float lengthSq() const { return x * x + y * y + z * z; }

	inline Vector3& normalize() { *this /= length(); return *this; }
	inline Vector3 normalized() const { return *this / length(); }

	Vector3 operator*( const Quaternion &q );
	Vector3& operator*=( const Quaternion &q );
};

inline Vector2 operator*( const Vector2& v, float x ) { return Vector2( v.x * x, v.y * x); }
inline Vector2 operator*( float x, const Vector2& v ) { return v * x; }
inline Vector3 operator*( const Vector3& v, float x ) { return Vector3( v.x * x, v.y * x, v.z * x ); }
inline Vector3 operator*( float x, const Vector3& v ) { return v * x; }

inline Vector3 min( const Vector3 &a, const Vector3 &b ) {
	return Vector3( min( a.x, b.x ), min( a.y, b.y ), min( a.z, b.z ) );
}
inline Vector3 max( const Vector3 &a, const Vector3 &b ) {
	return Vector3( max( a.x, b.x ), max( a.y, b.y ), max( a.z, b.z ) );
}
inline Vector3 min( const Vector3 &a, const Vector3 &b, const Vector3 &c ) {
	return min( a, min( b, c ) );
}
inline Vector3 max( const Vector3 &a, const Vector3 &b, const Vector3 &c ) {
	return max( a, max( b, c ) );
}

inline Vector3 min( const Vector3 &v, float val ) {
	return Vector3( min( v.x, val ), min( v.y, val ), min( v.z, val ) );
}
inline Vector3 max( const Vector3 &v, float val ) {
	return Vector3( max( v.x, val ), max( v.y, val ), max( v.z, val ) );
}

inline Vector3 clamp( const Vector3 &v, float minVal, float maxVal ) {
	return min( max( v, minVal ), maxVal );
}
inline Vector3 clamp01( const Vector3 &v ) {
	return clamp( v, 0.0f, 1.0f );
}

inline Vector3 clampPositive( const Vector3 &v ) {
	return max( v, 0.0f );
}

inline float dot( const Vector3& a, const Vector3&b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vector3 cross( const Vector3& a, const Vector3&b ) { return Vector3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x ); }
