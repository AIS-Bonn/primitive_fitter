#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>

#include <nimbro_primitive_fitter/primitive_fitter.h>
#include <nimbro_primitive_fitter/inertia.h>


#define NODE_NAME "nimbro_primitive_fitter"

int main(int argc, char **argv)
{
	std::string nodeName = NODE_NAME;
	ros::init(argc, argv, nodeName.c_str());

	ros::NodeHandle pvtNh("~" + nodeName); // private node handle

	ROS_INFO(" ********** NimbRo Primitive Fitter started ********** ");

	std::string in_filename;
	pvtNh.getParam("/" + nodeName + "/urdf_filename", in_filename);
	
	std::string out_filename("");
	pvtNh.getParam("/" + nodeName + "/output_filename", out_filename);
	
	if(out_filename.empty())
	{
		size_t lastindex = in_filename.find_last_of(".");
		out_filename = in_filename.substr(0, lastindex) + "_optimized.urdf.xacro";
	}

	bool fitter = true;
	pvtNh.getParam("/" + nodeName + "/use_fitter", fitter);

	bool inertia = false;	
	pvtNh.getParam("/" + nodeName + "/use_inertia", inertia);

	std::string capsule_filename("");
	
	if (inertia)
	{
		ROS_INFO("Inertia calculation started.");
		
		std::string mesh_type = "visual";
		bool visual = true;
		pvtNh.getParam("/" + nodeName + "/mesh_type", mesh_type);
		
		if (mesh_type == "visual")
		{
			visual = true;
			ROS_INFO("Using visual meshes.");
		}
		else if (mesh_type == "collision")
		{
			visual = false;
			ROS_INFO("Using collision meshes.");
		}
		else
		{
			ROS_WARN("PLEASE SET ROSPARAM /nimbro_primitive_fitter/mesh_type TO EITHER visual OR collsion -> Using 'visual'.");
			visual = true;
		}
		
		update_inertia(in_filename, out_filename, visual, pvtNh);
		in_filename = out_filename;
	}
	
	if (fitter)
	{
		std::string shape = "box";
		pvtNh.getParam("/" + nodeName + "/fit_shape", shape);
		VersatileFitter::Shape vf_shape;

		if (shape == "box" || shape == "Box")
		{
			ROS_INFO("Will try to fit URDF to Box.");
			vf_shape = VersatileFitter::Shape::Box;
		}
		else if (shape == "capsule" || shape == "Capsule")
		{
			ROS_INFO("Will try to fit URDF to Capsule.");
			vf_shape = VersatileFitter::Shape::Capsule;
		}
		else
		{
			ROS_ERROR("PLEASE SET ROSPARAM /nimbro_primitive_fitter/fit_shape TO EITHER box OR capsule");
			ROS_INFO("Will try to fit URDF to Box.");
			vf_shape = VersatileFitter::Shape::Box;
		}

		VersatileFitter vf;
		vf.fit(in_filename, out_filename, vf_shape, shape, pvtNh);
	}
	
	ros::shutdown();
	return 0;
}
