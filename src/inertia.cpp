#include <nimbro_primitive_fitter/primitive_fitter.h>
#include <nimbro_primitive_fitter/inertia.h>

#include <iostream>
#include <fstream>
#include <string>
#include <regex>

#include <ros/package.h>
#include <ros/ros.h>

#include <wrap/io_trimesh/import_stl.h>
#include <vcg/complex/algorithms/inertia.h>


std::string getFileExtension(std::string meshfile)
{
	std::string file_extension("");

	// Get file extension
	std::regex file_regex("([^\\\\.]+)(\\.)([^\\\\.]+)");
	std::smatch r_match;

	if (std::regex_search(meshfile, r_match, file_regex))
	{
		file_extension = r_match[3];
		ROS_DEBUG_STREAM( "Detected file extension ." << r_match[3] );
	}
	else
	{
		ROS_INFO( "NO FILE EXTENSION FOUND!" );
	}

	return file_extension;
}


void update_inertia(std::string in_filename, std::string out_filename, bool visual, const ros::NodeHandle nh)
{
	Urdf urdf(in_filename.c_str(), false);

	// Return if the urdf was not loaded
	if ( !urdf.loaded() )
	{
		ROS_ERROR( "URDF could not be loaded." );
		return;
	}

	bool hasMoreMesh;

	if (visual)
	{
		hasMoreMesh = urdf.hasMoreVisualMesh();
	}
	else
	{
		hasMoreMesh = urdf.hasMoreMesh();
	}

	while (hasMoreMesh)
	{
		std::string meshfile;

		if (visual)
		{
			meshfile = urdf.getNextVisualMesh();
			hasMoreMesh = urdf.hasMoreVisualMesh();
		}
		else
		{
			meshfile = urdf.getNextMesh();
			hasMoreMesh = urdf.hasMoreMesh();
		}

		std::string file_extension = getFileExtension(meshfile);

		if (file_extension == "stl" || file_extension == "STL")
		{
			ROS_DEBUG( "Handling STL file ..." );
			ROS_DEBUG_STREAM( "path: " << meshfile );

			// get global meshfile path
			url::Url filepath(meshfile);
			std::string pkgName(filepath.getPackageName());
			std::string pkgPath(ros::package::getPath(pkgName));
			std::string path(filepath.getRelativePath());
			std::string global_meshfile((pkgPath + path).c_str());
			ROS_INFO_STREAM( "global path: " << global_meshfile );

			// open STL file
			MyMesh mesh;
			int mask = 0;

			if(vcg::tri::io::ImporterSTL<MyMesh>::Open(mesh, global_meshfile.c_str(), mask))
			{
				ROS_INFO_STREAM( "Error reading file " << global_meshfile );
				return;
			}

			// read mass from file
			float mass = urdf.getMass(visual);

			// compute inertia and center of mass
			vcg::Matrix33f Inertia;
			vcg::tri::Inertia<MyMesh> I(mesh);
			I.InertiaTensor(Inertia);
			vcg::Point3f CenterOfMass = I.CenterOfMass();

			// scale inertia according to mass
			float volume = I.Mass();
			Inertia /= volume;
			Inertia *= mass;

			// set new values
			urdf.setCenterOfMass(CenterOfMass, visual);
			urdf.setInertia(Inertia, visual);
		}
		else
		{
			ROS_ERROR( "UNKNOWN FILE EXTENSION! (only .stl supported)" );
			return;
		}
	}

	if (urdf.save(out_filename.c_str()))
	{
		ROS_INFO( "Inertia updated successfully!" );
	}

	return;
}
