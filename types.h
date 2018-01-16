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
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel2;

// include and typedef for surface simplification algorithm
#include "surface_simplification_include.h"
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Surface_mesh;
typedef Surface_mesh::Point_3 SmPoint;
typedef Surface_mesh::Halfedge_handle SmHalfedge_handle;
typedef Surface_mesh::Vertex_handle SmVertex_handle;
namespace SMS = CGAL::Surface_mesh_simplification;
typedef SMS::Edge_profile<Surface_mesh> SmProfile;

#endif // ALG_VIS_TYPES_H
