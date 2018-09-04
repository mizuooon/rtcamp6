#pragma once

class Scene;
class Texture;
struct ObjectStructureIteratorHistory;

struct SampledRay {
	Vector3 d;
	Vector3 p;
	Vector3 n;
	Vector3 bsdf_cos_divided_p;
};

struct Material : public std::enable_shared_from_this<Material> {
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) = 0;
	virtual Vector3 getEmission() { return Vector3( 0.0f ); }

	std::shared_ptr<ParticipatingMedia> participatingMedia;
};

struct ParticipatingMedia : public Material {
	float absorptionCoefficient;
	float scatteringCoefficient;
	float extinctionCoefficient() { return absorptionCoefficient + scatteringCoefficient; }
	float transmittance( float t ) {
		return expf( -t * extinctionCoefficient() );
	}
	float pdf_t( float t ) { return extinctionCoefficient() * exp( -extinctionCoefficient() * t ); }
	float sampleDistance() {
		return -logf( randf() ) / extinctionCoefficient();
	}
};

struct Diffuse : public Material {
	Diffuse( const Vector3& albedo ) : albedo( albedo ), emission( Vector3( 0.0f ) ) {}
	Diffuse( const Vector3& albedo, const Vector3& emission ) : albedo( albedo ), emission( emission ) {}
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history );
	virtual Vector3 getEmission() { return emission; }

	Vector3 albedo;
	Vector3 emission;
};

struct DiffuseTextured : public Material {
	DiffuseTextured( std::shared_ptr<Texture> texture ) : texture( texture ) {}
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history );
	virtual Vector3 getEmission() { return Vector3( 0.0f ); }

	std::shared_ptr<Texture> texture;
};

struct DipoleSSS : public Material {
	DipoleSSS( const Vector3& albedo, float extinction, float refractiveIndex, float r_max ) : refractiveIndex( refractiveIndex ), r_max( r_max ) {
		scatteringCoefficient = albedo * extinction;
		absorptionCoefficient = Vector3( extinction ) - scatteringCoefficient;
	}
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history );
	virtual Vector3 getEmission() { return Vector3( 0.0f ); }
	Vector3 bssrdf( float r, const Vector3 &omega_i, const Vector3 &omega_o, const Vector3 &n ) const;
	float F_r( const Vector3 &i, const Vector3 &n ) const {
		const float eta = refractiveIndex;
		const float c = fabsf( dot( i, n ) );
		float g_sq = powf( eta, 2.0f ) - 1.0f + c * c;
		if ( g_sq <= 0.0f ) { return 1.0f; }
		float g = sqrtf( g_sq );
		return 0.5f * powf( ( g - c ) / ( g + c ), 2.0f ) * ( 1.0f + powf( ( c*( g + c ) - 1.0f ) / ( c*( g - c ) + 1.0f ), 2.0f ) );
	};

	float r_max;
	float refractiveIndex;
	Vector3 absorptionCoefficient;
	Vector3 scatteringCoefficient;
	Vector3 extinctionCoefficient() const { return absorptionCoefficient + scatteringCoefficient; }
};


struct NullSurface : public Material {
	NullSurface() {}

	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
		SampledRay sample;
		sample.p = intersection.p;
		sample.n = intersection.n;
		sample.d = in.d;
		sample.bsdf_cos_divided_p = 1.0f;
		return std::move( sample );
	}
	virtual Vector3 getEmission() { return Vector3( 0.0f ); }

	virtual Vector3 bsdf( const Vector3 &i, const Vector3 &x_i, const Vector3 &n_i, const Vector2 &uv_i, const Vector3 &o, const Vector3 &x_o, const Vector3& n_o ) const {
		return 0.0f;
	}
};

struct GGXRefraction : public Material {
	GGXRefraction( float refractiveIndex, float alpha_g ) : refractiveIndex( refractiveIndex ), alpha_g( alpha_g ) {}
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history );
	virtual Vector3 getEmission() { return Vector3( 0.0f ); }
	float F( const Vector3 &i, const Vector3 &m, float eta_t, float eta_i ) const {
		float c = fabsf( dot( i, m ) );
		float g_sq = powf( eta_t / eta_i, 2.0f ) - 1.0f + c * c;
		if ( g_sq <= 0.0f ) { return 1.0f; }
		float g = sqrtf( g_sq );
		return 0.5f * powf( ( g - c ) / ( g + c ), 2.0f ) * ( 1.0f + powf( ( c*( g + c ) - 1.0f ) / ( c*( g - c ) + 1.0f ), 2.0f ) );
	};
	float G1( const Vector3 &v, const Vector3 &m, const Vector3 &n ) const {
		float tan_theta_v_sq = 1.0f / powf( dot( n, v ), 2.0f ) - 1.0f;
		return clampPositive( dot( v, m ) / dot( v, n ) ) * 2.0f / ( 1.0f + sqrtf( 1.0f + alpha_g * alpha_g * tan_theta_v_sq ) );
	};
	float G( const Vector3 &i, const Vector3 &o, const Vector3 &m, const Vector3 &n ) const {
		return G1( i, m, n ) * G1( o, m, n );
	};
	float D( const Vector3 &m, const Vector3 &n ) const {
		float cos_theta_m = dot( m, n );
		float tan_theta_m_sq = 1.0f / powf( cos_theta_m, 2.0f ) - 1.0f;
		float alpha_g_sq = powf( alpha_g, 2.0f );
		return alpha_g_sq * clampPositive( cos_theta_m ) / ( PI * powf( cos_theta_m, 4.0f ) * powf( alpha_g_sq + tan_theta_m_sq, 2.0f ) );
	};

	float refractiveIndex;
	float alpha_g;
};

struct GGXReflection : public Material {
	GGXReflection( const Vector3 &albedo, float alpha_g ) : albedo( albedo ), alpha_g( alpha_g ) {}
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history );
	virtual Vector3 getEmission() { return Vector3( 0.0f ); }
	float G1( const Vector3 &v, const Vector3 &m, const Vector3 &n ) const {
		float tan_theta_v_sq = 1.0f / powf( dot( n, v ), 2.0f ) - 1.0f;
		return clampPositive( dot( v, m ) / dot( v, n ) ) * 2.0f / ( 1.0f + sqrtf( 1.0f + alpha_g * alpha_g * tan_theta_v_sq ) );
	};
	float G( const Vector3 &i, const Vector3 &o, const Vector3 &m, const Vector3 &n ) const {
		return G1( i, m, n ) * G1( o, m, n );
	};
	float D( const Vector3 &m, const Vector3 &n ) const {
		float cos_theta_m = dot( m, n );
		float tan_theta_m_sq = 1.0f / powf( cos_theta_m, 2.0f ) - 1.0f;
		float alpha_g_sq = powf( alpha_g, 2.0f );
		return alpha_g_sq * clampPositive( cos_theta_m ) / ( PI * powf( cos_theta_m, 4.0f ) * powf( alpha_g_sq + tan_theta_m_sq, 2.0f ) );
	};

	Vector3 albedo;
	float refractiveIndex;
	float alpha_g;
};

struct GGXTextured : public Material {
	// GGXReflection Ç∆ÇŸÇ⁄íÜêgàÍèè
	GGXTextured( std::shared_ptr<Texture> texture, float alpha_g ) :texture( texture ), alpha_g( alpha_g ) {}
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history );
	virtual Vector3 getEmission() { return Vector3( 0.0f ); }
	float G1( const Vector3 &v, const Vector3 &m, const Vector3 &n ) const {
		float tan_theta_v_sq = 1.0f / powf( dot( n, v ), 2.0f ) - 1.0f;
		return clampPositive( dot( v, m ) / dot( v, n ) ) * 2.0f / ( 1.0f + sqrtf( 1.0f + alpha_g * alpha_g * tan_theta_v_sq ) );
	};
	float G( const Vector3 &i, const Vector3 &o, const Vector3 &m, const Vector3 &n ) const {
		return G1( i, m, n ) * G1( o, m, n );
	};
	float D( const Vector3 &m, const Vector3 &n ) const {
		float cos_theta_m = dot( m, n );
		float tan_theta_m_sq = 1.0f / powf( cos_theta_m, 2.0f ) - 1.0f;
		float alpha_g_sq = powf( alpha_g, 2.0f );
		return alpha_g_sq * clampPositive( cos_theta_m ) / ( PI * powf( cos_theta_m, 4.0f ) * powf( alpha_g_sq + tan_theta_m_sq, 2.0f ) );
	};

	std::shared_ptr<Texture> texture;
	float alpha_g;
};

struct IsotopicMedia : public ParticipatingMedia {
	virtual SampledRay sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
		SampledRay sample;
		sample.p = intersection.p;
		sample.n = intersection.n;

		float r1 = randf();
		float r2 = randf();
		sample.d = Vector3( 2 * cosf( 2.0f * PI * r1 ) * sqrtf( r2 * ( 1.0f - r2 ) ),
							2 * sinf( 2.0f * PI * r1 ) * sqrtf( r2 * ( 1.0f - r2 ) ),
							1 - 2.0f * r2 );

		Vector3 phaseFunction = albedo * ( 1.0f / ( 4.0f * PI ) );
		float pdf_o = 1.0f / ( 4.0f * PI );

		sample.bsdf_cos_divided_p = transmittance( intersection.t ) * scatteringCoefficient * phaseFunction / ( pdf_o * pdf_t( intersection.t ) );

		return std::move( sample );
	}

	Vector3 albedo;
};
