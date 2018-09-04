#include "General.h"
#include "Texture.h"
#define STB_IMAGE_IMPLEMENTATION
#include "3rdparty/stb/stb_image.h"

Texture::Texture( const std::string &filename ) {
	loadFile( filename );
}

void Texture::loadFile( const std::string &filename ) {
	unsigned char* pixels = stbi_load( filename.c_str(), &width, &height, &bpp, 0 );

	data.clear();
	data.resize( width * height );
	for ( int y = 0; y < height; y++ ) {
		for ( int x = 0; x < width; x++ ) {
			int i = y * width + x;
			data[i] =
				Vector3( pixels[i * 3 + 0] / 255.0f,
						 pixels[i * 3 + 1] / 255.0f,
						 pixels[i * 3 + 2] / 255.0f );
		}
	}

	stbi_image_free( pixels );
}

Vector3 Texture::getTexel( const Vector2 &uv ) const {
	float x = clamp( uv.x * width, 0.0f, width - 1.0f );
	float y = clamp( uv.y * height, 0.0f, height - 1.0f );

	int xi1 = (int)( x - 0.5f );
	int yi1 = (int)( y - 0.5f );
	int xi2 = xi1 + 1;
	int yi2 = yi1 + 1;
	xi1 = clamp( xi1, 0, width - 1 );
	yi1 = clamp( yi1, 0, height - 1 );
	xi2 = clamp( xi2, 0, width - 1 );
	yi2 = clamp( yi2, 0, height - 1 );

	float xw2 = x - ( xi1 + 0.5f );
	float yw2 = y - ( yi1 + 0.5f );
	float xw1 = 1.0f - xw2;
	float yw1 = 1.0f - yw2;

	auto d = [&]( int x, int y ) { return data[y * width + x]; };

	return xw1 * yw1 * d( xi1, yi1 )
		 + xw1 * yw2 * d( xi1, yi2 )
		 + xw2 * yw1 * d( xi2, yi1 )
		 + xw2 * yw2 * d( xi2, yi2 );
}