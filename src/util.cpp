// Automatic capsule generator from URDF
// Author:  Arindam Roychoud
//          Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#include <nimbro_primitive_fitter/util.h>

#include <iostream>
#include <cstdio>
#include <string>
#include <algorithm>

#include <boost/foreach.hpp>

#include <pugixml.hpp>


namespace geometry
{

void matrix2rpy(const Eigen::Matrix3d m, Eigen::Vector3d &rpy)
{
	if (m(0, 0) == 0 || m(1, 0) == 0)
	{
		rpy(0) = 0;
		rpy(1) = M_PI / 2;
		rpy(2) = atan2(m(0, 1), m(1, 1));
	}
	else
	{
		rpy(0) = atan2(m(1, 0), m(0, 0));
		rpy(1) = atan2(-m(2, 0), std::sqrt(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0)) );
		rpy(2) = atan2(m(2, 1), m(2, 2));
	}
}


Eigen::Matrix4d create_transformation_matrix( Eigen::Vector3d rpy, Eigen::Vector3d xyz, Eigen::Vector3d scale)
{
	double ax, ay, az, tx, ty, tz, sx, sy, sz;
	ax = rpy ( 0 );
	ay = rpy ( 1 );
	az = rpy ( 2 );
	tx = xyz ( 0 );
	ty = xyz ( 1 );
	tz = xyz ( 2 );
	sx = scale ( 0 );
	sy = scale ( 1 );
	sz = scale ( 2 );

	Eigen::Affine3d rx = Eigen::Affine3d (
	                         Eigen::AngleAxisd ( ax, Eigen::Vector3d::UnitX() ) );
	Eigen::Affine3d ry = Eigen::Affine3d (
	                         Eigen::AngleAxisd ( ay, Eigen::Vector3d::UnitY() ) );
	Eigen::Affine3d rz = Eigen::Affine3d (
	                         Eigen::AngleAxisd ( az, Eigen::Vector3d::UnitZ() ) );
	Eigen::Affine3d r = rz * ry * rx;

	Eigen::Affine3d t ( Eigen::Translation3d ( Eigen::Vector3d ( tx, ty, tz ) ) );
	Eigen::Affine3d s ( Eigen::Scaling ( Eigen::Vector3d ( sx, sy, sz ) ) );

	Eigen::Matrix4d m = ( t * r * s ).matrix();

	return m;
}


Eigen::Vector3d rpyFromaxisAngles( Eigen::Vector3d rot_i )
{
	Eigen::Vector3d z ( Eigen::Vector3d::UnitZ() );

	Eigen::Vector3d a_n = rot_i.normalized();

	Eigen::Vector3d N = z.cross ( a_n ); // 	 default alignment of the cylinder is with the z-axis

	double sine = N.norm();
	double cosine = z.dot ( a_n );

	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Nx = Eigen::Matrix3d::Zero();

	Eigen::Vector3d x = N.normalized();

	Nx << 0, -x ( 2 ), x ( 1 ), x ( 2 ), 0, -x ( 0 ), -x ( 1 ), x ( 0 ), 0;

	Eigen::Matrix3d R = I + sine * Nx + ( 1.0 - cosine ) * ( Nx * Nx );

	double yaw = atan2 ( ( double ) R ( 1, 0 ), ( double ) R ( 0, 0 ) );
	double pitch = atan2 ( ( double ) - R ( 2, 0 ),
	                       ( double ) pow ( ( double ) ( R ( 2, 1 ) * R ( 2, 1 ) + R ( 2, 2 ) * R ( 2, 2 ) ), 0.5 ) );
	double roll = atan2 ( ( double ) R ( 2, 1 ), ( double ) R ( 2, 2 ) );

	Eigen::Vector3d rpy = Eigen::Vector3d ( roll, pitch, yaw );

	return rpy;
}


// Get a vector from a space separated const char
void getAsVector( const char* str, std::vector<double>* vec )
{
	std::vector<double> retVal;
	std::istringstream vertexStream( str );

	double val;

	while ( vertexStream >> val )
	{
		vec->push_back ( val );
	}
}

} // end of namespace geometry.


namespace url
{

url::Url::Url( std::string pkgUrl )
{
	this->pkgUrl = pkgUrl;
}


std::string  url::Url::getPackageName()
{
	std::string x ( this->pkgUrl );
	size_t pos = 10;		// ignore 'package://'
	size_t sp = x.find_first_of ( "/", pos );
	return x.substr ( pos, sp - pos );
}


std::string url::Url::getRelativePath()
{
	std::string x ( this->pkgUrl );
	size_t sp = x.find_first_of ( '/', 10 );
	std::string relPath ( x.begin() + sp, x.end() );
	return relPath;
}

} // end of namespace url

