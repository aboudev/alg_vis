////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "PCA_fitting.h"

#include <CGAL/centroid.h>
#include <CGAL/Subdivision_method_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "Color_256.h"

namespace Algs {

PCA_fitting::PCA_fitting() {}

void PCA_fitting::draw() {
  render_line();
  render_plane();
  render_centroid();
}

Vector PCA_fitting::normalize(const Vector& v)
{
  return v / std::sqrt(v*v);
}

void PCA_fitting::refine_loop()
{
  if (m_pPolyhedron == nullptr) {
    std::cout << "Load polyhedron first." << std::endl;
    return;
  }
  std::cout << "Loop subdivision...";
  CGAL::Subdivision_method_3::Loop_subdivision(*m_pPolyhedron, 1);
  std::cout << "done (" << m_pPolyhedron->size_of_facets() << " facets)" << std::endl;
}

void PCA_fitting::fit_triangles()
{
  if (m_pPolyhedron == nullptr) {
    std::cout << "Load polyhedron first." << std::endl;
    return;
  }

  std::cout << "Fit triangles...";

  std::list<Triangle> triangles;
  for (auto it = m_pPolyhedron->facets_begin(); it != m_pPolyhedron->facets_end(); ++it) {
    Polyhedron::Halfedge_handle he = it->halfedge();
    const Point& a = he->vertex()->point();
    const Point& b = he->next()->vertex()->point();
    const Point& c = he->next()->next()->vertex()->point();
    Triangle triangle(a,b,c);
    triangles.push_back(triangle);
  }

  m_centroid = CGAL::centroid(triangles.begin(),triangles.end());
  CGAL::linear_least_squares_fitting_3(triangles.begin(),
    triangles.end(), m_line, CGAL::Dimension_tag<2>()); 
  CGAL::linear_least_squares_fitting_3(triangles.begin(),
    triangles.end(), m_plane, CGAL::Dimension_tag<2>()); 

  std::cout << "done" << std::endl;
}

void PCA_fitting::fit_edges()
{
  if (m_pPolyhedron == nullptr) {
    std::cout << "Load polyhedron first." << std::endl;
    return;
  }

  std::cout << "Fit edges...";

  std::list<Segment> segments;
  Polyhedron::Edge_iterator he;
  for (he = m_pPolyhedron->edges_begin();
    he != m_pPolyhedron->edges_end();
    he++) {
    const Point& a = he->vertex()->point();
    const Point& b = he->opposite()->vertex()->point();
    Segment segment(a,b);
    segments.push_back(segment);
  }
  
  m_centroid = CGAL::centroid(segments.begin(),segments.end());
  CGAL::linear_least_squares_fitting_3(segments.begin(),
    segments.end(), m_line, CGAL::Dimension_tag<1>()); 
  CGAL::linear_least_squares_fitting_3(segments.begin(),
    segments.end(), m_plane, CGAL::Dimension_tag<1>()); 

  std::cout << "done" << std::endl;
}

void PCA_fitting::fit_vertices()
{
  if (m_pPolyhedron == nullptr) {
    std::cout << "Load polyhedron first." << std::endl;
    return;
  }

  std::cout << "Fit vertices...";

  std::list<Point> points;
  Polyhedron::Vertex_iterator v;
  for (v = m_pPolyhedron->vertices_begin();
    v != m_pPolyhedron->vertices_end();
    v++) {
    const Point& p = v->point();
    points.push_back(p);
  }
  
  m_centroid = CGAL::centroid(points.begin(),points.end());
  CGAL::linear_least_squares_fitting_3(points.begin(),
    points.end(), m_line, CGAL::Dimension_tag<0>()); 
  CGAL::linear_least_squares_fitting_3(points.begin(),
    points.end(), m_plane, CGAL::Dimension_tag<0>()); 

  std::cout << "done" << std::endl;
}

void PCA_fitting::render_plane()
{
  ::glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
  ::glLineWidth(3.0f);
  ::glColor3ub(255,0,0);
  ::glBegin(GL_QUADS);
  Point o = m_plane.projection(m_centroid);
  Point a = o + normalize(m_plane.base1()) + normalize(m_plane.base2());
  Point b = o + normalize(m_plane.base1()) - normalize(m_plane.base2());
  Point c = o - normalize(m_plane.base1()) - normalize(m_plane.base2());
  Point d = o - normalize(m_plane.base1()) + normalize(m_plane.base2());
  ::glVertex3d(a.x(),a.y(),a.z());
  ::glVertex3d(b.x(),b.y(),b.z());
  ::glVertex3d(c.x(),c.y(),c.z());
  ::glVertex3d(d.x(),d.y(),d.z());
  ::glEnd();
}

void PCA_fitting::render_line()
{
  ::glLineWidth(3.0f);
  ::glColor3ub(0,0,255);
  ::glBegin(GL_LINES);
  Point o = m_line.projection(m_centroid);
  Point a = o + normalize(m_line.to_vector());
  Point b = o - normalize(m_line.to_vector());
  ::glVertex3d(a.x(),a.y(),a.z());
  ::glVertex3d(b.x(),b.y(),b.z());
  ::glEnd();
}

void PCA_fitting::render_centroid()
{
  ::glPointSize(10.0f);
  ::glColor3ub(0,128,0);
  ::glBegin(GL_POINTS);
  ::glVertex3d(m_centroid.x(),m_centroid.y(),m_centroid.z());
  ::glEnd();
}

}
