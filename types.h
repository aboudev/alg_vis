#ifndef ALG_VIS_TYPES_H
#define ALG_VIS_TYPES_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <boost/iterator/transform_iterator.hpp> 

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::FT FT;
typedef Kernel::Line_3 Line;
typedef Kernel::Point_3 Point;
typedef Kernel::Plane_3 Plane;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Segment_3 Segment;
typedef Kernel::Triangle_3 Triangle;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Bbox_3 Bbox;


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
// plane regularization in CGAL 4.9
#include <CGAL/regularize_planes.h>

// Type declarations
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel2;
typedef std::pair<Kernel2::Point_3, Kernel2::Vector_3>       Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
// In Shape_detection_traits the basic types, i.e., Point and Vector types
// as well as iterator type and property maps, are defined.
typedef CGAL::Shape_detection_3::Shape_detection_traits
<Kernel2, Pwn_vector, Point_map, Normal_map>                  Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
typedef CGAL::Shape_detection_3::Plane<Traits>               RansacPlane;

// include and typedef for surface simplification algorithm
#include "surface_simplification_include.h"
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Surface_mesh;
typedef Surface_mesh::Point_3 SmPoint;
typedef Surface_mesh::Halfedge_handle SmHalfedge_handle;
typedef Surface_mesh::Vertex_handle SmVertex_handle;
namespace SMS = CGAL::Surface_mesh_simplification;
typedef SMS::Edge_profile<Surface_mesh> SmProfile;

#endif // ALG_VIS_TYPES_H
