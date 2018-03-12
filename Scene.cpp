#include "Scene.h"
#include "Surface_simplification.h"
#include "Shape_detection.h"
#include "Horizontal_plane_detection.h"
#include "Unit_normal_detection.h"
#include "Symmetric_normal_detection.h"
#include "Ridge_detection.h"

#include <iostream>
#include <fstream>

#ifdef _WIN32
#include <windows.h>
#endif
#include <gl/gl.h>

#include <CGAL/IO/Polyhedron_iostream.h>

Scene::Scene() :
  m_pPolyhedron(nullptr),
  m_view_polyhedron(false),
  m_surface_simplification(nullptr),
  m_shape_detection(nullptr),
  m_horizontal_plane_detection(nullptr),
  m_unit_normal_detection(nullptr),
  m_symmetric_normal_detection(nullptr),
  m_ridge_detection(nullptr) {
}

Scene::~Scene() {
  if (m_pPolyhedron)
    delete m_pPolyhedron;

  if (m_surface_simplification)
    delete m_surface_simplification;
  if (m_shape_detection)
    delete m_shape_detection;
  if (m_horizontal_plane_detection)
    delete m_horizontal_plane_detection;
  if (m_unit_normal_detection)
    delete m_unit_normal_detection;
  if (m_symmetric_normal_detection)
    delete m_symmetric_normal_detection;
  if (m_ridge_detection)
    delete m_ridge_detection;
}

int Scene::open(const std::string &fname)
{
  std::cerr << "Opening file " << fname << std::endl;

  std::ifstream ifs(fname);
  if (!ifs.is_open()) {
    std::cerr << "unable to open file" << std::endl;
    return -1;
  }

  if (m_pPolyhedron != nullptr)
    delete m_pPolyhedron;

  // allocate new polyhedron
  m_pPolyhedron = new Polyhedron;
  ifs >> *m_pPolyhedron;
  if (!ifs) {
    std::cerr << "invalid OFF file" << std::endl;

    delete m_pPolyhedron;
    m_pPolyhedron = nullptr;

    return -1;
  }

  m_bbox = CGAL::bbox_3(m_pPolyhedron->points_begin(), m_pPolyhedron->points_end());
  m_view_polyhedron = true;

  return 0;
}

int Scene::surface_simplification(const std::string &fname)
{
  delete_all_algorithms();

  m_surface_simplification = new Algs::Surface_simplification();
  m_surface_simplification->simplify(fname);

  // update viewing bbox
  // m_bbox = m_surface_simplification->bbox();
  // m_view_polyhedron = false;

  return 0;
}

int Scene::shape_detection(const std::string &fname, const Params::Shape_detection &params)
{
  delete_all_algorithms();

  m_shape_detection = new Algs::Shape_detection();
  m_shape_detection->detect(fname, params);

  // update viewing bbox
  m_bbox = m_shape_detection->bbox();
  m_view_polyhedron = false;

  return 0;
}

int Scene::horizontal_plane_detection(const std::string &fname, const Params::Shape_detection &params)
{
  delete_all_algorithms();

  m_horizontal_plane_detection = new Algs::Horizontal_plane_detection();
  m_horizontal_plane_detection->detect(fname, params);

  // update viewing bbox
  m_bbox = m_horizontal_plane_detection->bbox();
  m_view_polyhedron = false;

  return 0;
}

int Scene::unit_normal_detection(const std::string &fname, const Params::Shape_detection &params)
{
  delete_all_algorithms();

  m_unit_normal_detection = new Algs::Unit_normal_detection();
  m_unit_normal_detection->detect(fname, params);

  // update viewing bbox
  m_bbox = m_unit_normal_detection->bbox();
  m_view_polyhedron = false;

  return 0;
}

int Scene::symmetric_normal_detection(
  const std::string &fname,
  const Params::Shape_detection &params,
  const bool is_constrained)
{
  delete_all_algorithms();

  m_symmetric_normal_detection = new Algs::Symmetric_normal_detection();
  m_symmetric_normal_detection->detect(fname, params, is_constrained);

  // update viewing bbox
  m_bbox = m_symmetric_normal_detection->bbox();
  m_view_polyhedron = false;

  return 0;
}

int Scene::ridge_detection(const std::string &fname)
{
  delete_all_algorithms();

  m_ridge_detection = new Algs::Ridge_detection();
  m_ridge_detection->detect(fname);

  // update viewing bbox
  m_bbox = m_ridge_detection->bbox();
  m_view_polyhedron = false;

  return 0;
}

void Scene::draw()
{
  if (m_view_polyhedron)
    render_polyhedron();

  // if (m_surface_simplification)
  //   m_surface_simplification->draw();

  if (m_shape_detection)
    m_shape_detection->draw();

  if (m_horizontal_plane_detection)
    m_horizontal_plane_detection->draw();

  if (m_unit_normal_detection)
    m_unit_normal_detection->draw();

  if (m_symmetric_normal_detection)
    m_symmetric_normal_detection->draw();

  if (m_ridge_detection)
    m_ridge_detection->draw();
}

void Scene::render_polyhedron()
{
  if (!m_pPolyhedron)
    return;

  // draw black edges
  ::glDisable(GL_LIGHTING);
  ::glColor3ub(0, 0, 0);
  ::glLineWidth(1.0f);
  ::glBegin(GL_LINES);
  for (auto he = m_pPolyhedron->edges_begin(); he != m_pPolyhedron->edges_end(); ++he) {
    const Point_3 &a = he->vertex()->point();
    const Point_3 &b = he->opposite()->vertex()->point();
    ::glVertex3d(a.x(), a.y(), a.z());
    ::glVertex3d(b.x(), b.y(), b.z());
  }
  ::glEnd();
}

void Scene::delete_all_algorithms()
{
  if (m_surface_simplification)
    delete m_surface_simplification;
  m_surface_simplification = nullptr;
  if (m_shape_detection)
    delete m_shape_detection;
  m_shape_detection = nullptr;
  if (m_horizontal_plane_detection)
    delete m_horizontal_plane_detection;
  m_horizontal_plane_detection = nullptr;
  if (m_unit_normal_detection)
    delete m_unit_normal_detection;
  m_unit_normal_detection = nullptr;
  if (m_symmetric_normal_detection)
    delete m_symmetric_normal_detection;
  m_symmetric_normal_detection = nullptr;
  if (m_ridge_detection)
    delete m_ridge_detection;
  m_ridge_detection = nullptr;
}
