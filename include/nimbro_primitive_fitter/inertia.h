#pragma once

#include <string>
#include <vcg/complex/complex.h>


class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public vcg::UsedTypes<	vcg::Use<MyVertex>   ::AsVertexType,
		vcg::Use<MyEdge>     ::AsEdgeType,
		vcg::Use<MyFace>     ::AsFaceType> {};

class MyVertex  : public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags  > {};
class MyFace    : public vcg::Face< MyUsedTypes, vcg::face::FFAdj, vcg::face::Normal3f, vcg::face::VertexRef, vcg::face::BitFlags > {};
class MyEdge    : public vcg::Edge<MyUsedTypes> {};
class MyMesh    : public vcg::tri::TriMesh< std::vector<MyVertex>, std::vector<MyFace> , std::vector<MyEdge>  > {};


std::string getFileExtension(std::string meshfile);

/*
 * Reads URDF and updates inertia values and center of mass for each link.
 * 'urdf_filename': Path of the URDF
 * 'visual': If true, visual meshes are used for calculation.
 *           If false, collision meshes are used.
 */
void update_inertia(std::string in_filename, std::string out_filename, bool visual, const ros::NodeHandle nh);
