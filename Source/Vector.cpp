#include "General.h"
#include "Vector.h"
#include "Quaternion.h"

Vector3 Vector3::operator*( const Quaternion &q ) {
	return q * (*this);
}

Vector3& Vector3::operator*=( const Quaternion &q ) {
	Vector3 v = q * ( *this );
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}