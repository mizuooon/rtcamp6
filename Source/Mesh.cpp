
#include "Mesh.h"
#include "ObjectStructure.h"
#include "Material.h"
#include "3rdparty/OBJ_Loader.h"
#include "Quaternion.h"

void Mesh::loadFile( std::string filename ) {
	objl::Loader loader;
	loader.LoadFile( filename.c_str() );

	// マテリアルをここで生成して割り当てる突貫仕様

	auto material_diffuse = std::make_shared<Diffuse>( Vector3( 0.2f, 0.2f, 0.2f ) );
	auto material_mame = std::make_shared<DipoleSSS>( Vector3( 0.6f, 0.3f, 0.1f ), 30.0f, 1.35f, 0.1f );
	auto material_shiratama = std::make_shared<DipoleSSS>( Vector3( 1.0f, 1.0f, 1.0f ), 20.0f, 1.35f, 0.2f );
	auto material_mikan = std::make_shared<DipoleSSS>( Vector3( 0.68f, 0.68f, 0.16f ), 10.0f, 1.35f, 0.4f );
	auto material_spoon = std::make_shared<GGXReflection>( Vector3( 0.15f, 0.15f, 0.15f ), 0.1f );
	auto material_kanten = std::make_shared<GGXRefraction>( 0.7f, 0.001f );
	auto media_kanten = std::make_shared<IsotopicMedia>();
	media_kanten->albedo = Vector3(1.0f,1.0f,0.98f);
	float a = 0.6f;
	float extinction = 5.0f;
	media_kanten->absorptionCoefficient = extinction * (1-a);
	media_kanten->scatteringCoefficient = extinction * a;
	material_kanten->participatingMedia = media_kanten;

	auto material_sara_in = std::make_shared<DipoleSSS>( Vector3( 0.15f, 0.22f, 0.15f ), 50.0f, 2.0f, 0.3f );
	auto material_sara_out = material_sara_in;

	auto material_cup = std::make_shared<DipoleSSS>( Vector3( 0.18f, 0.16f, 0.12f ), 50.0f, 2.0f, 0.3f );;
	auto material_mitsu = std::make_shared<GGXRefraction>( 1.2f, 0.001f );
	auto media_mitsu = std::make_shared<IsotopicMedia>();
	media_mitsu->albedo = Vector3( 1.0f, 1.0f, 0.9f );
	a = 0.01f;
	extinction = 20.0f;
	media_mitsu->absorptionCoefficient = extinction * ( 1 - a );
	media_mitsu->scatteringCoefficient = extinction * a;
	material_mitsu->participatingMedia = media_mitsu;

	for (const auto &mesh : loader.LoadedMeshes) {
		std::shared_ptr<Material> material = material_diffuse;

		if ( mesh.MeshName.substr( 0, 6 ) == std::string("kanten") ) {
			material = material_kanten;
		}
		if ( mesh.MeshName.substr( 0, 4 ) == std::string( "mame" ) ) {
			material = material_mame;
		}
		if ( mesh.MeshName.substr( 0, 5 ) == std::string( "spoon" ) ) {
			material = material_spoon;
		}
		if ( mesh.MeshName.substr( 0, 7 ) == std::string( "sara_in" ) ) {
			material = material_sara_in;
		}
		if ( mesh.MeshName.substr( 0, 8 ) == std::string( "sara_out" ) ) {
			material = material_sara_out;
		}
		if ( mesh.MeshName.substr( 0, 9 ) == std::string( "shiratama" ) ) {
			material = material_shiratama;
		}
		if ( mesh.MeshName.substr( 0, 5 ) == std::string( "mikan" ) ) {
			material = material_mikan;
		}
		if ( mesh.MeshName.substr( 0, 3 ) == std::string( "cup" ) ) {
			material = material_cup;
		}
		if ( mesh.MeshName.substr( 0, 5 ) == std::string( "mitsu" ) ) {
			material = material_mitsu;
		}

		const auto &indices = mesh.Indices;
		const auto &vertices = mesh.Vertices;

		triangles.reserve( triangles.size() + indices.size() / 3 );
		for ( int i = 0; i < indices.size() / 3; i++ ) {
			const auto &v0 = vertices[indices[i * 3 + 0]];
			const auto &v1 = vertices[indices[i * 3 + 1]];
			const auto &v2 = vertices[indices[i * 3 + 2]];
			auto triangle = std::make_shared<Triangle>();

			auto f = []( objl::Vertex in ) {
				Vertex v;
				v.p.x = in.Position.X;
				v.p.y = in.Position.Y;
				v.p.z = in.Position.Z;
				v.n.x = in.Normal.X;
				v.n.y = in.Normal.Y;
				v.n.z = in.Normal.Z;

				v.texCoord.x = in.TextureCoordinate.X;
				v.texCoord.y = in.TextureCoordinate.Y;
				return v;
			};

			triangle->v[0] = f( v0 );
			triangle->v[1] = f( v1 );
			triangle->v[2] = f( v2 );

			triangle->material = material;

			triangles.push_back( triangle );
		}
	}

}

std::shared_ptr<MeshInstance> Mesh::createInstance( const Transform &t ) {
	return std::make_shared<MeshInstance>(shared_from_this(), t);
}


MeshInstance::MeshInstance( std::shared_ptr<Mesh> mesh, const Transform &t ) {
	triangles.reserve( mesh->getTriangles().size() );
	for ( int i = 0; i < mesh->getTriangles().size(); i++ ) {
		auto triangle = std::make_shared<Triangle>();
		auto origTri = std::static_pointer_cast<Triangle>( mesh->getTriangles()[i] );
		*triangle = *origTri;

		triangle->v[0].p *= t.rotation;
		triangle->v[1].p *= t.rotation;
		triangle->v[2].p *= t.rotation;
		triangle->v[0].n *= t.rotation;
		triangle->v[1].n *= t.rotation;
		triangle->v[2].n *= t.rotation;

		triangle->v[0].p *= t.scale;
		triangle->v[1].p *= t.scale;
		triangle->v[2].p *= t.scale;
		triangle->v[0].p += t.position;
		triangle->v[1].p += t.position;
		triangle->v[2].p += t.position;

		triangle->material = origTri->material;

		triangles.push_back( triangle );
	}

	aabb = ::getAABB( triangles );
}

std::shared_ptr<ObjectStructure> MeshInstance::buildObjectStructure() const {
	return ::buildObjectStructure( triangles );
}