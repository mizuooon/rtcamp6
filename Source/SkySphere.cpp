
#include "SkySphere.h"
#include "3rdparty/hdrloader.h"

void SkySphere::loadHDRFile ( const std::string &filename ) {
	HDRLoaderResult result;
	HDRLoader::load ( filename.c_str (), result );
	width = result.width;
	height = result.height;
	data.reserve ( width * height );
	for ( int i = 0; i < width * height; i++ ) {
		Vector3 col;
		col.x = result.cols[i * 3 + 0];
		col.y = result.cols[i * 3 + 1];
		col.z = result.cols[i * 3 + 2];
		data.push_back ( std::move ( col ) );
	}
}

Vector3 SkySphere::getRadiance ( Vector3 d ) {
	d *= -1;
	float r = ( 1.0f / PI ) * acosf ( d.z ) / sqrtf ( d.x * d.x + d.y * d.y );
	float u = d.x * r;
	float v = d.y * r;

	int x = ( u + 1.0f ) / 2.0f * width;
	int y = ( v + 1.0f ) / 2.0f * height;
	x = clamp ( x, 0, width - 1 );
	y = clamp ( y, 0, height - 1 );

	return data[x + y * width];
}
