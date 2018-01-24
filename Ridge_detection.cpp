////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-24
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Ridge_detection.h"

#include <iostream>
#include <fstream>

#include <CGAL/Ridges.h> 
#include <CGAL/Umbilics.h>

#ifdef _WIN32
#include <windows.h>
#endif
#include <gl/gl.h>
#include "Color_256.h"

namespace Algs {

void Ridge_detection::detect(const std::string &fname)
{
  typedef Polyhedron PolyhedralSurf;
  typedef Kernel::FT FT;
  typedef Kernel::Point_3 Point_3;
  typedef Kernel::Vector_3 Vector_3;
  typedef boost::graph_traits<PolyhedralSurf>::vertex_descriptor vertex_descriptor;
  typedef boost::graph_traits<PolyhedralSurf>::vertex_iterator vertex_iterator;
  typedef boost::graph_traits<PolyhedralSurf>::face_descriptor face_descriptor;
  typedef std::map<vertex_descriptor, FT> VertexFT_map;
  typedef boost::associative_property_map< VertexFT_map > VertexFT_property_map;
  typedef std::map<vertex_descriptor, Vector_3> VertexVector_map;
  typedef boost::associative_property_map<VertexVector_map> VertexVector_property_map;

  //RIDGES
  typedef CGAL::Ridge_line<PolyhedralSurf> Ridge_line;
  typedef CGAL::Ridge_approximation<
    PolyhedralSurf,
    VertexFT_property_map,
    VertexVector_property_map > Ridge_approximation;

  //UMBILICS
  typedef CGAL::Umbilic<PolyhedralSurf> Umbilic;
  typedef CGAL::Umbilic_approximation<
    PolyhedralSurf,
    VertexFT_property_map,
    VertexVector_property_map > Umbilic_approximation;

  //create property maps
  VertexFT_map vertex_k1_map, vertex_k2_map,
    vertex_b0_map, vertex_b3_map,
    vertex_P1_map, vertex_P2_map;
  VertexVector_map vertex_d1_map, vertex_d2_map;
  VertexFT_property_map vertex_k1_pm(vertex_k1_map), vertex_k2_pm(vertex_k2_map),
    vertex_b0_pm(vertex_b0_map), vertex_b3_pm(vertex_b3_map),
    vertex_P1_pm(vertex_P1_map), vertex_P2_pm(vertex_P2_map);
  VertexVector_property_map vertex_d1_pm(vertex_d1_map), vertex_d2_pm(vertex_d2_map);

  // load triangle mesh
  PolyhedralSurf P;

  //compute differential quantities with the jet fitting package

  //initialize the property maps

  //Ridges
  //--------------------------------------------------------------------------
  Ridge_approximation ridge_approximation(P,
    vertex_k1_pm, vertex_k2_pm,
    vertex_b0_pm, vertex_b3_pm,
    vertex_d1_pm, vertex_d2_pm,
    vertex_P1_pm, vertex_P2_pm);

  std::vector<Ridge_line *> ridge_lines;
  std::back_insert_iterator<std::vector<Ridge_line *> > ii(ridge_lines);
  CGAL::Ridge_order tag_order = CGAL::Ridge_order_3;
  //Find MAX_RIDGE, MIN_RIDGE, CREST or all ridges
  ridge_approximation.compute_max_ridges(ii, tag_order);
  ridge_approximation.compute_min_ridges(ii, tag_order);
  ridge_approximation.compute_crest_ridges(ii, tag_order);

  // UMBILICS
  //--------------------------------------------------------------------------
  const double umb_size = 0.0;
  Umbilic_approximation umbilic_approximation(P,
    vertex_k1_pm, vertex_k2_pm,
    vertex_d1_pm, vertex_d2_pm);
  std::vector<Umbilic *> umbilics;
  std::back_insert_iterator<std::vector<Umbilic *> > umb_it(umbilics);
  umbilic_approximation.compute(umb_it, umb_size);
}

void Ridge_detection::draw() {}

} // Algs
