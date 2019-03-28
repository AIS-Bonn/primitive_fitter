// Automatic capsule generator from URDF
// Author:  Arindam Roychoud
//          Diego Rodriguez <rodriguez@ais.uni-bonn.de>

#pragma once

#include <Eigen/Geometry>


namespace geometry
{
void matrix2rpy( const Eigen::Matrix3d m, Eigen::Vector3d &rpy );

Eigen::Matrix4d create_transformation_matrix( Eigen::Vector3d rpy, Eigen::Vector3d xyz, Eigen::Vector3d scale );

Eigen::Vector3d rpyFromaxisAngles( Eigen::Vector3d rot_i );

// Get a vector from a space separated const char
void getAsVector( const char* str, std::vector<double>* vec );

} // end of namespace geometry.


namespace url
{

class Url
{
public:
	Url( std::string pkgUrl );
	std::string getPackageName();
	std::string getRelativePath();
	
private:
	std::string pkgUrl;
};

} // end of namespace url
