// Automatic capsule generator from URDF or XACRO files
// Authors:
//          André Brandenburger <s6anbran@uni-bonn.de>
//          Diego Rodriguez <rodriguez@ais.uni-bonn.de>
//          Arindam Roychoud

#pragma once

#include <eigen3/Eigen/Geometry>

#include <pugixml.hpp>

#include <roboptim/capsule/fitter.hh>

#include <nimbro_primitive_fitter/util.h>

#include "../contrib/approx_mvbb/include/ApproxMVBB/ComputeApproxMVBB.hpp"

#include <ros/ros.h>

#include <vcg/complex/complex.h>

#include <fstream>

using namespace roboptim;
using namespace roboptim::capsule;

class Urdf;



/**
    Abstract ShapeFit class for fitting different shapes
*/
class ShapeFit
{
public:
	/**
	Writes the fitted model into URDF file.

	@param lastLinkNode The XML node where collision-block shall be appended.
	*/
	virtual void writeUrdf(boost::shared_ptr<pugi::xml_node> lastLinkNode) = 0;

	/**
	Writes the fitted model into URDF file. Uses Xacro Macros.

	@param lastLinkNode The XML node where collision-block shall be appended.
	*/
	virtual void writeUrdfXacro(boost::shared_ptr<pugi::xml_node> lastLinkNode) = 0;
};


//******************************************************************************************************************
/*
 * The best fitting capsule that's going to replace the mesh in the urdf collision properties.
 * */
class CapsuleFit: public ShapeFit
{
public:

	Eigen::Vector3d endpoint1() const { return m_endpoint1; };

	Eigen::Vector3d endpoint2() const { return m_endpoint2; };

	double radius() const { return m_radius; };

	/*
	 * Capsule length. (Length of the cylinder between the two half-sphere endpoints. This does not include the sphere radii.
	 */
	double getLength();

	/*
	 * The capsule's mid-point.
	 */
	Eigen::Vector3d getMidpoint();

	/*
	 * The capsule's orientation in space.
	 */
	Eigen::Vector3d getRotation();

	/*
	 * Compute the best fit capsule from the mesh vertices, mesh orientation and position as defined in the urdf file and also in the mesh's collada file / STL file
	 */
	CapsuleFit(std::vector<double> points,
	           Eigen::Vector3d translation,
	           Eigen::Vector3d rotation,
	           Eigen::Vector3d scale,
	           Eigen::Matrix4d transformation);

	/**
	Writes the fitted model into URDF file.

	@param lastLinkNode The XML node where collision-block shall be appended.
	Should be the last link in the XML.
	*/
	void writeUrdf(boost::shared_ptr<pugi::xml_node> lastLinkNode);

	/**
	Writes the fitted model into URDF file. Uses Xacro Macros.

	@param lastLinkNode The XML node where collision-block shall be appended.
	Should be the last link in the XML.
	*/
	void writeUrdfXacro(boost::shared_ptr< pugi::xml_node > lastLinkNode);

	/**
	Writes Xacro Macro for the used structure. Empty since Box natively supported

	@param doc The XML document to which the macro shall be added.
	*/
	static void writeXacroMacro(const pugi::xml_document* doc);

private:
	void transformParameters(const Eigen::Matrix4d &transform, const Eigen::Matrix4d &localTransform);

	/*
	 * Capsule endpoint1
	 */
	Eigen::Vector3d m_endpoint1;

	/*
	 * Capsule endpoint2
	 */
	Eigen::Vector3d m_endpoint2;

	/*
	 * Capsule radius
	 */
	double m_radius;

	std::vector<double> m_points;

	double m_length;

	Eigen::Vector3d m_midpoint;
	Eigen::Vector3d m_rot;
};


//******************************************************************************************************************
/*
* Class for finding an Optimal Oriented Bounding Box (OOBB)
* for a given point set.
*/
class BoxFit: public ShapeFit
{
public:
	BoxFit(std::vector<double> points,
	       Eigen::Vector3d translation,
	       Eigen::Vector3d rotation,
	       Eigen::Vector3d scale,
	       Eigen::Matrix4d transformation,
	       const ros::NodeHandle nh
	      );

	/**
	Writes the fitted model into URDF file.

	@param lastLinkNode The XML node where collision-block shall be appended.
	Should be the last link in the XML.
	*/
	void writeUrdf(boost::shared_ptr<pugi::xml_node> lastLinkNode);

	/**
	Writes the fitted model into URDF file. Same as writeUrdf, since boxes natively supported.

	@param lastLinkNode The XML node where collision-block shall be appended.
	Should be the last link in the XML.
	*/
	void writeUrdfXacro(boost::shared_ptr<pugi::xml_node> lastLinkNode);

	/**
	Writes Xacro Macro for the used structure. Empty since Box natively supported

	@param doc The XML document to which the macro shall be added.
	*/
	static void writeXacroMacro(const pugi::xml_document* doc) { }; // Empty since natively supported

private:
	/**
	Read the points and process them in a way suitable for the fitter.

	@param points The flat(!) vector containing all points in form (x1, y1, z1, x2, y2, z2,...)
	*/
	void readPoints(const std::vector<double> points);
	/**
	Checks if point-vector is OK and can be read.

	@param points The flat(!) vector containing all points in form (x1, y1, z1, x2, y2, z2,...)
	*/
	bool checkPoints(const std::vector<double> points);

	// Output of the Optimal Oriented Bounding Box fitting
	ApproxMVBB::OOBB m_oobb;

	// Point set used by the fitter - read in via readPoints(std::vector<double>)
	Eigen::Matrix<double, 3, Eigen::Dynamic> m_points;

	Eigen::Vector3d m_trans;
	Eigen::Vector3d m_rot;
	Eigen::Vector3d m_box_center;
	Eigen::Vector3d m_box_size;
	
	double m_roll;
	double m_pitch;
	double m_yaw;
};


//******************************************************************************************************************
/*
* Organizer of the fitting processes.
* Can fit different kind of shapes.
*/
class VersatileFitter
{
public:
	VersatileFitter();

	/**
	 * All currently supported shapes.
	 */
	enum Shape {Capsule, Box};

	/**
	Reads URDF and successively fits all links to the defined shape.

	@param urdf_filename The path of the URDF that shall be fitted.
	@param shape Shape that shall be fitted to the meshes.
	@see Shape
	*/
	void fit(std::string urdf_filename, std::string &output_filename, Shape shape, std::string shape_name, const ros::NodeHandle nh);
private:
	std::string getFileExtension(std::string meshfile);
	void loadMesh(std::string meshfile);
};


//******************************************************************************************************************
/*
 * Urdf file operations. Can work with xacro.
 * Add collision nodes as xacros or plain urdf.
 *
 */
class Urdf
{
public:
	/*
	 * Get next mesh file defining collision properties.
	 */
	std::string getNextMesh();

	/*
	 * Get next mesh file defining visual properties.
	 */
	std::string getNextVisualMesh();

	/*
	 * True: File has more mesh files defining collision properties.
	 */
	bool hasMoreMesh();

	/*
	 * True: File has more mesh files defining visual properties.
	 */
	bool hasMoreVisualMesh();

	/*
	 * Constructor: Accept the urdf file path (mandatory). Add collision tags as xacro (optional)
	 */
	Urdf(const char* filepath, bool xacro);

	/*
	 * Replace mesh based collision file capsule based plain urdf collision tags. Adds three tags that approximate a capsule: <spher>, <cylinder>, <sphere>.
	 */
	void replaceMesh(boost::shared_ptr<ShapeFit> shape);

	/*
	 * Replace mesh based collision with capsule based xacro collision tags.
	 */
	void replaceMeshWithXacro(boost::shared_ptr<ShapeFit> shape);

	/*
	 * Get the original mesh rotation as a 3D Eulerian rotation vector.
	 */
	Eigen::Vector3d getMeshRotation() const;

	/*
	 * Get the original mesh translation as a 3D vector.
	 */
	Eigen::Vector3d getMeshTranslation() const;

	/*
	 * Get the original mesh scale as a 3D vector.
	 */
	Eigen::Vector3d getMeshScale() const;

	/*
	 * Save the updated urdf file at the supplied location.
	 */

	bool save(const char* path);

	/*
	 * Returns mass of current mesh.
	 * If visual = true, visual mesh will be used, otherwise collision mesh is used.
	 * Mesh must be previously extracted using getNextMesh() or getNextVisualMesh().
	 */
	float getMass(bool visual);

	/*
	 * Replaces Inertia for current mesh.
	 * If visual = true, visual mesh will be used, otherwise collision mesh is used.
	 * Mesh must be previously extracted using getNextMesh() or getNextVisualMesh(), accordingly.
	 */
	void setInertia(vcg::Matrix33f Inertia, bool visual);

	/*
	 * Replaces center of mass for current mesh.
	 * If visual = true, visual mesh will be used, otherwise collision mesh is used.
	 * Mesh must be previously extracted using getNextMesh() or getNextVisualMesh(), accordingly.
	 */
	void setCenterOfMass(vcg::Point3f CenterOfMass, bool visual);

	/*
	 * Whether to add new collision tags as xacros.
	 */
	bool isXacro();

	bool loaded() { return m_URDFLoaded; };

protected:
	void addXacroDef();

	const char* m_filepath;
	const std::string m_collision_path = "/robot/link[*]/collision/geometry/mesh";
	const std::string m_visual_path = "/robot/link[*]/visual/geometry/mesh";

	const pugi::xpath_node* m_it;
	const pugi::xpath_node* m_visual_it;
	pugi::xpath_node_set m_nodes;
	pugi::xpath_node_set m_visual_nodes;

	pugi::xml_node m_lastLinkNode;
	pugi::xml_node m_lastCollisionNode;
	pugi::xml_node m_lastMeshNode;

	pugi::xml_node m_lastVisualLinkNode;
	pugi::xml_node m_lastVisualNode;
	pugi::xml_node m_lastVisualMeshNode;

	pugi::xml_document m_doc;

	bool m_xacro;
	bool m_URDFLoaded = false;
};


//******************************************************************************************************************
/*
 *  Abstract meshfile superclass to enable easier future extensions
 */
class MeshFile
{
public:
	/**
	Read the points from the mesh file and return them in a flat vector of type double.

	@returns points The flat(!) vector containing all points in form (x1, y1, z1, x2, y2, z2,...)
	*/
	virtual std::vector<double> getPoints() = 0;

	/**
	Return the transformation of the mesh like it is neccessary for e.g. collada files.

	@returns matrix A transformation matrix in homogenous coordinates
	*/
	virtual Eigen::Matrix4d getTransformationMatrix() = 0;
};


//******************************************************************************************************************
/*
 * *.dae or collada file operations. Read triangle mesh vertices and the original mesh orientation.
 */
class Dae: public MeshFile
{
public:
	/*
	 * Open file handle to supplied dae file.
	 */
	Dae(std::string filename);

	/*
	 * Get mesh vertices as a list.
	 */
	std::vector<double> getPoints();

	/*
	 * Get the mesh model's orientation in space.
	 */
	Eigen::Matrix4d getTransformationMatrix();

private:
	std::string m_filename;
	pugi::xml_document m_doc;
};


//******************************************************************************************************************
/*
*.stl file operations. Read vertices from binary STL files
*/
class Stl: public MeshFile
{
public:
	/*
	 * Open file handle to supplied STL file.
	 */
	Stl(std::string filename);

	/*
	 * Get mesh vertices as a list.
	 */
	std::vector<double> getPoints();

	/*
	 * Get the mesh model's orientation in space. (here just identity)
	 */
	Eigen::Matrix4d getTransformationMatrix() { return Eigen::Matrix4d::Identity(); };

private:
	float parseFloat();
	void dumpPoint();
	
	std::ifstream m_filestream;

	unsigned int m_num_faces;
};
