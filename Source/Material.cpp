#include "General.h"
#include "Geometry.h"
#include "Material.h"
#include "Vector.h"
#include "GeometryUtils.h"
#include "Texture.h"
#include "Scene.h"

SampledRay Diffuse::sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
	float r1 = randf();
	float r2 = randf();
	BasisVector basis = genBasisVector( intersection.n );
	SampledRay sample;
	sample.p = intersection.p;
	sample.n = intersection.n;
	sample.d = basis.vector( sqrtf( r2 ), cosf( 2 * PI * r1 ) * sqrtf( 1 - r2 ), sinf( 2 * PI * r1 ) * sqrtf( 1 - r2 ) );

	const Vector3& n = intersection.n;
	const Vector3& i = -in.d;
	const Vector3& o = sample.d;

	sample.bsdf_cos_divided_p = ( dot( o, n ) > 0.0f ? albedo / PI : 0.0f ) * PI;
	return std::move( sample );
}

SampledRay DiffuseTextured::sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
	float r1 = randf();
	float r2 = randf();
	BasisVector basis = genBasisVector( intersection.n );
	SampledRay sample;
	sample.p = intersection.p;
	sample.n = intersection.n;
	sample.d = basis.vector( sqrtf( r2 ), cosf( 2 * PI * r1 ) * sqrtf( 1 - r2 ), sinf( 2 * PI * r1 ) * sqrtf( 1 - r2 ) );

	const Vector3& n = intersection.n;
	const Vector3& i = -in.d;
	const Vector3& o = sample.d;

	sample.bsdf_cos_divided_p = ( dot( o, n ) > 0.0f ? texture->getTexel( intersection.uv ) / PI : 0.0f ) * PI;
	return std::move( sample );
}

SampledRay DipoleSSS::sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {

	const Vector3& omega_i = -in.d;

	float probeOffset = 0.2f;

	float r1 = randf();
	float r2 = randf();
	BasisVector basis = genBasisVector( intersection.n );
	Vector3 v =
		( 1 - powf( r1, 2 ) ) * cosf( r2 * PI * 2 ) * basis.e2
		+ ( 1 - powf( r1, 2 ) ) * sinf( r2 * PI * 2 ) * basis.e3;

	Ray probe;
	probe.o = intersection.p + r_max * v + probeOffset * basis.e1;
	probe.d = -intersection.n;
	auto probeIntsct = scene->getIntersection( probe, history, nullptr );

	Vector3 x_o, omega_o;
	Vector3 n_o;
	if ( probeIntsct ) {
		x_o = probeIntsct->p;
		n_o = probeIntsct->n;
	} else {
		x_o = intersection.p;
		n_o = intersection.n;
	}

	{
		float r1 = randf();
		float r2 = randf();
		BasisVector basis = genBasisVector( n_o );
		omega_o = basis.vector( sqrtf( r2 ), cosf( 2 * PI * r1 ) * sqrtf( 1 - r2 ), sinf( 2 * PI * r1 ) * sqrtf( 1 - r2 ) );
	}
	SampledRay sample;

	if ( randf() < F_r( omega_i, intersection.n ) ) {
		sample.p = intersection.p;
		sample.n = intersection.n;
		sample.d = -omega_i + dot( omega_i, intersection.n ) * 2 * intersection.n;
		sample.bsdf_cos_divided_p = 1.0f;
	} else {
		sample.p = x_o;
		sample.n = n_o;
		sample.d = omega_o;
		sample.bsdf_cos_divided_p = bssrdf( ( intersection.p - x_o ).length(), omega_i, omega_o, intersection.n ) * PI / ( dot( intersection.n, n_o ) / ( PI * r_max * r_max ) );
	}

	return std::move( sample );
}

Vector3 DipoleSSS::bssrdf( float r, const Vector3 &omega_i, const Vector3 &omega_o, const Vector3 &n ) const {
	const float eta = refractiveIndex;
	const float g = 0.0f;
	const Vector3 sigma_a = absorptionCoefficient;
	const Vector3 sigma_s = scatteringCoefficient;
	const Vector3 sigma_t = extinctionCoefficient();
	const Vector3 sigma_s_prime = sigma_s * ( 1.0f - g );
	const Vector3 sigma_t_prime = sigma_s_prime + sigma_a;
	const Vector3 sigma_tr( sqrtf( 3 * sigma_a.x * sigma_t_prime.x ),
							sqrtf( 3 * sigma_a.y * sigma_t_prime.y ),
							sqrtf( 3 * sigma_a.z * sigma_t_prime.z ) );
	const Vector3 alpha_prime = sigma_s_prime / sigma_t_prime;
	const float F_dr = -1.440f / powf( eta, 2.0f ) + 0.710f / eta + 0.668f + 0.0636f*eta;
	const float A = ( 1 + F_dr ) / ( 1 - F_dr );
	const Vector3 D = Vector3( 1.0f ) / ( 3 * sigma_t_prime );
	const Vector3 z_r = Vector3( 1.0f ) / sigma_t_prime;
	const Vector3 z_v = z_r + 4 * A*D;
	auto R_d = [&]( float r ) {
		const Vector3 d_r = z_r;
		const Vector3 d_v = z_v;
		auto f = [&]( const Vector3 &d ) {
			return Vector3(
				expf( -sigma_tr.x * d.x ) / ( sigma_t_prime.x * powf( d.x, 3 ) ),
				expf( -sigma_tr.z * d.y ) / ( sigma_t_prime.y * powf( d.y, 3 ) ),
				expf( -sigma_tr.y * d.z ) / ( sigma_t_prime.z * powf( d.z, 3 ) )
			);
		};
		return alpha_prime / ( 4.0f * PI )
			* ( z_r * ( sigma_tr * d_r + 1 ) * f( d_r )
				+ z_v * ( sigma_tr * d_v + 1 ) * f( d_v )
				);
	};
	auto F_t = [&]( const Vector3 &i ) {
		return 1.0f - F_r( i, n );
	};
	auto S_d = [&]( float r, const Vector3 &omega_i, const Vector3 &omega_o ) {
		return 1 / PI * F_t( omega_i ) * R_d( r ) * F_t( omega_o );
	};

	return S_d( r, omega_i, omega_o );
}

SampledRay GGXRefraction::sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
	float r1 = randf();
	float r2 = randf();
	SampledRay sample;
	sample.p = intersection.p;
	sample.n = intersection.n;
	Vector3 i = intersection.i;
	Vector3 n = intersection.n;
	BasisVector basis = genBasisVector( n );

	float theta_m = atanf( alpha_g * sqrt( r1 ) / sqrt( 1.0f - r1 ) );
	float phai_m = 2.0f * PI * r2;
	const Vector3 m = basis.vector( cosf( theta_m ), sinf( theta_m )*cosf( phai_m ), sinf( theta_m )*sinf( phai_m ) );

	float eta_t, eta_i;
	if ( dot( i, n ) < 0.0f ) {
		eta_i = 1.0f;
		eta_t = refractiveIndex;
	} else {
		eta_i = refractiveIndex;
		eta_t = 1.0f;
	}

	float fresnel = F( i, m, eta_t, eta_i );
	if ( randf() <= fresnel ) {
		sample.d = ( 2.0f * fabsf( dot( i, m ) ) * m - i ).normalize();
	} else {
		float c = dot( i, m );
		float eta = eta_i / eta_t;
		sample.d = ( eta * c - ( dot( i, n ) > 0.0f ? 1.0f : -1.0f ) * sqrtf( 1.0f + eta * ( c*c - 1.0f ) ) ) * m - eta * i;
		sample.d.normalize();
	}

	const Vector3& o = sample.d;

	float weight = fabsf( dot( i, m ) * G( i, o, m, n ) / ( dot( i, n ) * dot( m, n ) ) );
	sample.bsdf_cos_divided_p = Vector3( weight );


	return std::move( sample );
}

SampledRay GGXReflection::sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
	float r1 = randf();
	float r2 = randf();
	SampledRay sample;
	sample.p = intersection.p;
	sample.n = intersection.n;
	Vector3 i = intersection.i;
	Vector3 n = intersection.n;
	BasisVector basis = genBasisVector( n );

	float theta_m = atanf( alpha_g * sqrt( r1 ) / sqrt( 1.0f - r1 ) );
	float phai_m = 2.0f * PI * r2;
	const Vector3 m = basis.vector( cosf( theta_m ), sinf( theta_m )*cosf( phai_m ), sinf( theta_m )*sinf( phai_m ) );

	sample.d = ( 2.0f * fabsf( dot( i, m ) ) * m - i ).normalize();

	const Vector3& o = sample.d;

	float weight = fabsf( dot( i, m ) * G( i, o, m, n ) / ( dot( i, n ) * dot( m, n ) ) );
	sample.bsdf_cos_divided_p = albedo * Vector3( weight );

	return std::move( sample );
}

SampledRay GGXTextured::sampleRay( const Ray& in, const Intersection& intersection, std::shared_ptr<Scene> scene, std::shared_ptr<ObjectStructureIteratorHistory> history ) {
	float r1 = randf();
	float r2 = randf();
	SampledRay sample;
	sample.p = intersection.p;
	sample.n = intersection.n;
	Vector3 i = intersection.i;
	Vector3 n = intersection.n;
	BasisVector basis = genBasisVector( n );

	float theta_m = atanf( alpha_g * sqrt( r1 ) / sqrt( 1.0f - r1 ) );
	float phai_m = 2.0f * PI * r2;
	const Vector3 m = basis.vector( cosf( theta_m ), sinf( theta_m )*cosf( phai_m ), sinf( theta_m )*sinf( phai_m ) );

	sample.d = ( 2.0f * fabsf( dot( i, m ) ) * m - i ).normalize();

	const Vector3& o = sample.d;

	float weight = fabsf( dot( i, m ) * G( i, o, m, n ) / ( dot( i, n ) * dot( m, n ) ) );
	sample.bsdf_cos_divided_p = texture->getTexel( intersection.uv ) * Vector3( weight );

	return std::move( sample );
}