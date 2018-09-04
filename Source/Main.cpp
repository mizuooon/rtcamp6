
#include "General.h"

#include <iostream>
#include <fstream>
#include <chrono>
#include <omp.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma warning(push)
#pragma warning(disable : 4996)
#include "3rdparty/stb/stb_image_write.h"
#pragma warning(pop)

#include "Geometry.h"
#include "Scene.h"
#include "Material.h"
#include "Mesh.h"
#include "PathTracer.h"
#include "Quaternion.h"
#include "Texture.h"

struct Camera {
	Vector3 position;
	Vector3 eye;
	Vector3 up;
	float horizontalFOV;
	float aspect;

	Ray getRay( float u, float v ) {
		// u [-1, 1]
		// v [-1, 1]

		float verticalFOV = horizontalFOV * aspect;
		Vector3 right = cross( eye, up );

		Vector3 d = ( eye + u * tanf( horizontalFOV / 2.0f * ToRad ) * right + v * tanf( verticalFOV / 2.0f * ToRad ) * up ).normalize();

		Ray ray;
		ray.o = position;
		ray.d = d;
		ray.depth = 1;

		return std::move( ray );
	}
};

struct ToneMapping {
	static float toneMap( float x ) {
		// とりあえずガンマ補正だけ
		return powf( clamp01( x ), 1.0f / 2.0f );
	}

};



int main() {

	auto start_time = std::chrono::system_clock::now();

	const int w = 800;
	const int h = 600;

	const int sampling = 4096; // 適当に大きい数字

	int outputCount = 0;
	int outputInterval = 15;
	int timeLimit = 120;


	// ---- シーン用意
	auto scene = std::make_shared<Scene>();

	// テーブル
	auto material_table = std::make_shared<GGXTextured>( std::make_shared<Texture>("Scene/wood.jpg"), 0.04f );
	std::shared_ptr<Triangle> triangle1, triangle2;
	triangle1 = std::make_shared<Triangle>();
	triangle2 = std::make_shared<Triangle>();
	float table_size = 7;
	triangle1->v[0].p = Vector3( -table_size/2, -0.02f, -table_size / 2 );
	triangle1->v[1].p = Vector3( table_size / 2, -0.02f, table_size / 2 );
	triangle1->v[2].p = Vector3( table_size / 2, -0.02f, -table_size / 2 );
	triangle2->v[0].p = Vector3( -table_size / 2, -0.02f, -table_size / 2 );
	triangle2->v[1].p = Vector3( -table_size / 2, -0.02f, table_size / 2 );
	triangle2->v[2].p = Vector3( table_size / 2, -0.02f, table_size / 2 );
	triangle1->material = material_table;
	triangle2->material = material_table;
	triangle1->calcNormal();
	triangle2->calcNormal();
	triangle1->v[0].texCoord = Vector2( 0, 0 );
	triangle1->v[1].texCoord = Vector2( 1, 1 );
	triangle1->v[2].texCoord = Vector2( 1, 0 );
	triangle2->v[0].texCoord = Vector2( 0, 0 );
	triangle2->v[1].texCoord = Vector2( 0, 1 );
	triangle2->v[2].texCoord = Vector2( 1, 1 );
	scene->addObject( triangle1 );
	scene->addObject( triangle2 );

	// 照明
	triangle1 = std::make_shared<Triangle>();
	triangle2 = std::make_shared<Triangle>();
	float s = 16;
	float power = 5;
	float light_z = -5;
	Vector3 lightColor = Vector3(1.0f, 1.0f, 0.8f) * power;
	triangle1->v[0].p = Vector3( 0 - s, 20.0f, light_z - s );
	triangle1->v[1].p = Vector3( 0 + s, 20.0f, light_z - s );
	triangle1->v[2].p = Vector3( 0 + s, 20.0f, light_z + s );
	triangle2->v[0].p = Vector3( 0 - s, 20.0f, light_z - s );
	triangle2->v[1].p = Vector3( 0 + s, 20.0f, light_z + s );
	triangle2->v[2].p = Vector3( 0 - s, 20.0f, light_z + s );
	triangle1->material = std::make_shared<Diffuse>( Vector3( 0.0f ), lightColor );
	triangle2->material = std::make_shared<Diffuse>( Vector3( 0.0f ), lightColor );
	triangle1->calcNormal();
	triangle2->calcNormal();
	scene->addObject( triangle1 );
	scene->addObject( triangle2 );
	scene->addExplicitLight( triangle1 );
	scene->addExplicitLight( triangle2 );

	// みつ豆
	auto mesh = std::make_shared<Mesh>();
	mesh->loadFile( "Scene/mitsumame.obj" );
	auto meshInstance = mesh->createInstance(
		Transform(
		Vector3( 0, 0, 0 ),
		Vector3( 1 ),
		Quaternion::quaternionRotationAxis( Vector3( 0, 1, 0 ), 0.0f ) )
	);
	scene->addObject( meshInstance );

	Camera camera;
	camera.aspect = (float)h / w;
	camera.horizontalFOV = 35.0f;
	camera.position = Vector3( -0.6f, 10, -6.5f );
	camera.eye = Vector3( 0, -1.5f, 1 ).normalize();
	camera.up = Vector3( 0, 1, 0 );

	// ---- BVH 作る
	auto start_time_tmp = std::chrono::system_clock::now();

	printf( "Start buillding data structure.\n" );
	scene->buildObjectStructure();
	printf( "Finish buillding data structure.\n" );

	auto current_time_tmp = std::chrono::system_clock::now();
	printf( "Elapsed Time : %f\n", std::chrono::duration_cast<std::chrono::milliseconds>( current_time_tmp - start_time_tmp ).count() / 1000.0f );


	// ---- レンダリング
	start_time_tmp = std::chrono::system_clock::now();
	printf( "Start rendering.\n" );

	std::vector<Vector3> radiances( w*h );
	std::vector<uint8_t> result( w * h * 3 );

	std::shared_ptr<PathTracer> pathTracer = std::make_shared<BSDFSamplingPathTracer>();
	PathTracer::russianRouretteProbability = 0.95f;
	PathTracer::originOffset = 0.00001f;

	int sampleCount = 0;
	for ( sampleCount = 0; sampleCount < sampling; sampleCount++ ) {
#pragma omp parallel for schedule(dynamic, 1)
		for ( int i = 0; i < w * h; i++ ) {
			try {
				int x = i % w;
				int y = i / w;
				float u = ( (float)x / w - 0.5f ) * 2.0f;
				float v = -( (float)y / h - 0.5f ) * 2.0f;

				Vector3 &radiance = radiances[i];
				Ray ray = camera.getRay( u, v );

				pathTracer->evalRadiance( scene, &ray );
				radiance += *ray.radiance;

			}
			catch ( std::exception &e ) {
				fprintf( stderr, "%s\n", e.what() );
				// たぶんもっかいやれば動くやろ
				continue;
			}
		}
		auto current_time = std::chrono::system_clock::now();
		int elapsedSec = (int)std::chrono::duration_cast<std::chrono::seconds>( current_time - start_time ).count();
		if ( elapsedSec>= outputInterval * (outputCount+1) ) {
			for ( int i = 0; i < w * h; i++ ) {
				Vector3 radiance = radiances[i];
				radiance /= ( sampleCount + 1.0f );
				result[i * 3 + 0] = (uint8_t)( ToneMapping::toneMap( radiance.x ) * 255 );
				result[i * 3 + 1] = (uint8_t)( ToneMapping::toneMap( radiance.y ) * 255 );
				result[i * 3 + 2] = (uint8_t)( ToneMapping::toneMap( radiance.z ) * 255 );
			}
			
			char outputCountStr[] = "000";
			sprintf_s( outputCountStr, 4, "%03d", outputCount );
			std::string filename = "" + std::string( outputCountStr ) + std::string( ".png" );
			stbi_write_png( filename.c_str(), w, h, 3, result.data(), w * 3 );
			++outputCount;
		}

		float timePerSample = (float) elapsedSec / (sampleCount + 1);
		printf( "%d sample (%d %%) | %d sec | %f sec/sample\r", sampleCount+1, (int)( (float)( sampleCount + 1 ) / sampling * 100 ), elapsedSec, timePerSample );

		if ( elapsedSec + timePerSample >= timeLimit ) {  // 次のサンプルが間に合わなさそうだったら終わる
			++sampleCount;
			break;
		}
	}
	printf( "\n" );

	for ( int i = 0; i < w * h; i++ ) {
		Vector3 &radiance = radiances[i];
		radiance /= (float)sampleCount;
		result[i * 3 + 0] = (uint8_t)( ToneMapping::toneMap( radiance.x ) * 255 );
		result[i * 3 + 1] = (uint8_t)( ToneMapping::toneMap( radiance.y ) * 255 );
		result[i * 3 + 2] = (uint8_t)( ToneMapping::toneMap( radiance.z ) * 255 );
	}

	printf( "Finish rendeirng.\n" );
	current_time_tmp = std::chrono::system_clock::now();
	printf( "Elapsed Time : %f\n", std::chrono::duration_cast<std::chrono::milliseconds>( current_time_tmp - start_time_tmp ).count() / 1000.0f );

	{
		auto current_time = std::chrono::system_clock::now();
		int elapsedSec = (int)std::chrono::duration_cast<std::chrono::seconds>( current_time - start_time ).count();
		char outputCountStr[] = "000";
		sprintf_s( outputCountStr, 4, "%03d", outputCount );
		std::string filename = "" + std::string( outputCountStr ) + std::string( ".png" );
		stbi_write_png( filename.c_str(), w, h, 3, result.data(), w * 3 );
	}

	return 0;
}