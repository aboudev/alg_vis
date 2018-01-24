////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-24
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Ridge_detection.h"
#include "PolyhedralSurf_rings.h"

#include <iostream>
#include <fstream>

#include <CGAL/Kernel/global_functions.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <CGAL/Ridges.h>
#include <CGAL/Umbilics.h>

#ifdef _WIN32
#include <windows.h>
#endif
#include <gl/gl.h>
#include "Color_256.h"

// property maps
typedef boost::property_map<Surface_mesh, boost::vertex_point_t>::type VertexPoint_property_map;
typedef Surface_mesh::Property_map<vertex_descriptor, FT> VertexFT_property_map;
typedef Surface_mesh::Property_map<vertex_descriptor, Vector_3> VertexVector_property_map;
typedef Surface_mesh::Property_map<face_descriptor, Vector_3> FaceVector_property_map;

typedef T_PolyhedralSurf_rings<Surface_mesh> Poly_rings;
typedef CGAL::Monge_via_jet_fitting<Kernel> Monge_via_jet_fitting;
typedef Monge_via_jet_fitting::Monge_form Monge_form;

// RIDGES
typedef CGAL::Ridge_line<Surface_mesh> Ridge_line;
typedef CGAL::Ridge_approximation<
  Surface_mesh,
  VertexFT_property_map,
  VertexVector_property_map> Ridge_approximation;

// UMBILICS
typedef CGAL::Umbilic<Surface_mesh> Umbilic;
typedef CGAL::Umbilic_approximation<
  Surface_mesh,
  VertexFT_property_map,
  VertexVector_property_map> Umbilic_approximation;

// default fct parameter values and global variables
unsigned int d_fitting = 3;
unsigned int d_monge = 3;
unsigned int nb_rings = 0;//seek min # of rings to get the required #pts
unsigned int nb_points_to_use = 0;//
CGAL::Ridge_order tag_order = CGAL::Ridge_order_3;
double umb_size = 2;
bool verbose = false;
unsigned int min_nb_points = (d_fitting + 1) * (d_fitting + 2) / 2;

// property maps
VertexPoint_property_map vpm;
FaceVector_property_map fvm;
VertexFT_property_map vertex_k1_pm, vertex_k2_pm,
  vertex_b0_pm, vertex_b3_pm,
  vertex_P1_pm, vertex_P2_pm;
VertexVector_property_map vertex_d1_pm, vertex_d2_pm;

/*!
 * \brief Compute face normal.
 */
void compute_facets_normal(const Surface_mesh &P);

Vector_3 compute_facets_average_unit_normal(const Surface_mesh& tm, vertex_descriptor v);

/*!
 * \brief Gather points around the vertex v using rings on the polyhedralsurf.
 * The collection of points resorts to 3 alternatives:
 * 1. the exact number of points to be used
 * 2. the exact number of rings to be used
 * 3. nothing is specified
 */
void gather_fitting_points(
  vertex_descriptor v,
  std::vector<Point_3> &in_points,
  Poly_rings &poly_rings);

/*!
 * \brief Use the jet_fitting package and the class Poly_rings to compute differential quantities.
 */
void compute_differential_quantities(Surface_mesh &P, Poly_rings &poly_rings);

namespace Algs {

void Ridge_detection::detect(const std::string &fname)
{
  // load triangle mesh
  std::ifstream ifs(fname);
  if (!ifs.is_open()) {
    std::cerr << "Error: failed to open " << fname << std::endl;
    return;
  }
  m_mesh = Surface_mesh();
  ifs >> m_mesh;
  ifs.close();

  if (min_nb_points > num_vertices(m_mesh)) {
    std::cerr << "not enough points in the model" << std::endl;
    return;
  }

  //initialize the property maps
  vpm = get(CGAL::vertex_point, m_mesh);
  fvm = m_mesh.add_property_map<face_descriptor, Vector_3>("f:n", Vector_3(0, 0, 0)).first;
  //initialize Polyhedral data : normal of facets
  compute_facets_normal(m_mesh);

  m_bbox = get(vpm, *vertices(m_mesh).first).bbox();
  BOOST_FOREACH(const vertex_descriptor v, vertices(m_mesh))
    m_bbox += get(vpm, v).bbox();

  vertex_k1_pm = m_mesh.add_property_map<vertex_descriptor, FT>("v:k1", 0).first;
  vertex_k2_pm = m_mesh.add_property_map<vertex_descriptor, FT>("v:k2", 0).first;
  vertex_b0_pm = m_mesh.add_property_map<vertex_descriptor, FT>("v:b0", 0).first;
  vertex_b3_pm = m_mesh.add_property_map<vertex_descriptor, FT>("v:b3", 0).first;
  vertex_P1_pm = m_mesh.add_property_map<vertex_descriptor, FT>("v:P1", 0).first;
  vertex_P2_pm = m_mesh.add_property_map<vertex_descriptor, FT>("v:P2", 0).first;
  vertex_d1_pm = m_mesh.add_property_map<vertex_descriptor, Vector_3>("v:d1", Vector_3(0, 0, 0)).first;
  vertex_d2_pm = m_mesh.add_property_map<vertex_descriptor, Vector_3>("v:d2", Vector_3(0, 0, 0)).first;

  //create a Poly_rings object
  Poly_rings poly_rings(m_mesh);

  //compute differential quantities with the jet fitting package
  std::cout << "Compute differential quantities via jet fitting..." << std::endl;
  compute_differential_quantities(m_mesh, poly_rings);

  //Ridges
  //--------------------------------------------------------------------------
  std::cout << "Compute ridges..." << std::endl;
  Ridge_approximation ridge_approximation(m_mesh,
    vertex_k1_pm, vertex_k2_pm,
    vertex_b0_pm, vertex_b3_pm,
    vertex_d1_pm, vertex_d2_pm,
    vertex_P1_pm, vertex_P2_pm);

  std::vector<Ridge_line *> ridge_lines;
  //Find MAX_RIDGE, MIN_RIDGE, CREST or all ridges
  // ridge_approximation.compute_max_ridges(std::back_inserter(ridge_lines), tag_order);
  // ridge_approximation.compute_min_ridges(std::back_inserter(ridge_lines), tag_order);
  ridge_approximation.compute_crest_ridges(std::back_inserter(ridge_lines), tag_order);

  // or with the global function
  // CGAL::compute_max_ridges(m_mesh,
  //   vertex_k1_pm, vertex_k2_pm,
  //   vertex_b0_pm, vertex_b3_pm,
  //   vertex_d1_pm, vertex_d2_pm,
  //   vertex_P1_pm, vertex_P2_pm,
  //   std::back_inserter(ridge_lines), tag_order);

  // to rendering data
  m_ridges.clear();
  std::cout << "#ridges " << ridge_lines.size() << std::endl;
  for (const auto &rl : ridge_lines) {
    std::vector<Point_3> ridge;
    for (const auto &rhe : *(rl->line())) {
      const Vector_3 p = get(vpm, source(rhe.first, m_mesh)) - CGAL::ORIGIN;
      const Vector_3 q = get(vpm, target(rhe.first, m_mesh)) - CGAL::ORIGIN;
      const Point_3 pt = CGAL::ORIGIN + (p * rhe.second + (1.0 - rhe.second) * q);
      ridge.push_back(pt);
    }
    m_ridges.push_back(ridge);

    delete rl;
  }

  // UMBILICS
  //--------------------------------------------------------------------------
  std::cout << "Compute umbilics..." << std::endl;
  Umbilic_approximation umbilic_approximation(m_mesh,
    vertex_k1_pm, vertex_k2_pm,
    vertex_d1_pm, vertex_d2_pm);
  std::vector<Umbilic *> umbilics;
  std::back_insert_iterator<std::vector<Umbilic *> > umb_it(umbilics);
  umbilic_approximation.compute(umb_it, umb_size);

  // to rendering data
  std::cout << "#umbilics " << umbilics.size() << std::endl;
  m_umbilics.clear();
  for (const auto &u : umbilics) {
    m_umbilics.push_back(get(vpm, u->vertex()));

    delete u;
  }
}

void Ridge_detection::draw()
{
  ::glEnable(GL_LIGHTING);
  ::glColor3ub(128, 128, 128);
  ::glBegin(GL_TRIANGLES);
  BOOST_FOREACH(const face_descriptor f, faces(m_mesh)) {
    const Vector_3 &n = get(fvm, f);
    ::glNormal3d(n.x(), n.y(), n.z());

    halfedge_descriptor h = halfedge(f, m_mesh);
    const Point_3 &p0 = get(vpm, source(h, m_mesh));
    const Point_3 &p1 = get(vpm, target(h, m_mesh));
    const Point_3 &p2 = get(vpm, target(next(h, m_mesh), m_mesh));
    ::glVertex3d(p0.x(), p0.y(), p0.z());
    ::glVertex3d(p1.x(), p1.y(), p1.z());
    ::glVertex3d(p2.x(), p2.y(), p2.z());
  }
  ::glEnd();

  ::glDisable(GL_LIGHTING);
  ::glColor3ub(255, 0, 0);
  ::glLineWidth(2.0);
  for (const auto &r : m_ridges) {
    ::glBegin(GL_LINE_STRIP);
    for (const auto &p : r)
      ::glVertex3d(p.x(), p.y(), p.z());
    ::glEnd();
  }

  ::glDisable(GL_LIGHTING);
  ::glColor3ub(255, 0, 0);
  ::glPointSize(5.0);
  ::glBegin(GL_POINTS);
  for (const auto &p : m_umbilics)
    ::glVertex3d(p.x(), p.y(), p.z());
  ::glEnd();
}

} // Algs

void compute_facets_normal(const Surface_mesh &P) {
  BOOST_FOREACH(face_descriptor f, faces(P)) {
    halfedge_descriptor h = halfedge(f, P);
    const Point_3 &p0 = get(vpm, source(h, P));
    const Point_3 &p1 = get(vpm, target(h, P));
    const Point_3 &p2 = get(vpm, target(next(h, P), P));
    put(fvm, f, CGAL::unit_normal(p0, p1, p2));
  }
}

Vector_3 compute_facets_average_unit_normal(const Surface_mesh& tm, vertex_descriptor v)
{
  Vector_3 sum(0.0, 0.0, 0.0);
  BOOST_FOREACH(face_descriptor f, faces_around_target(halfedge(v, tm), tm)) {
    if (f != boost::graph_traits<Surface_mesh>::null_face())
      sum = sum + get(fvm, f);
  }
  sum = sum / std::sqrt(sum * sum);

  return sum;
}

void gather_fitting_points(
  vertex_descriptor v,
  std::vector<Point_3> &in_points,
  Poly_rings &poly_rings)
{
  //container to collect vertices of v on the Surface_mesh
  std::vector<vertex_descriptor> gathered;
  //initialize
  in_points.clear();

  //OPTION -p nb_points_to_use, with nb_points_to_use != 0. Collect
  //enough rings and discard some points of the last collected ring to
  //get the exact "nb_points_to_use"
  if (nb_points_to_use != 0) {
    poly_rings.collect_enough_rings(v, nb_points_to_use, gathered);
    if (gathered.size() > nb_points_to_use)
      gathered.resize(nb_points_to_use);
  }
  else { // nb_points_to_use=0, this is the default and the option -p is not considered;
    // then option -a nb_rings is checked. If nb_rings=0, collect
    // enough rings to get the min_nb_points required for the fitting
    // else collect the nb_rings required
    if (nb_rings == 0)
      poly_rings.collect_enough_rings(v, min_nb_points, gathered);
    else
      poly_rings.collect_i_rings(v, nb_rings, gathered);
  }

  //store the gathered points
  for (const auto &v : gathered)
    in_points.push_back(get(vpm, v));
}

void compute_differential_quantities(Surface_mesh &P, Poly_rings &poly_rings)
{
  //container for approximation points
  std::vector<Point_3> in_points;

  BOOST_FOREACH(const vertex_descriptor v, vertices(P)) {
    //initialize
    in_points.clear();
    Monge_form monge_form;
    Monge_via_jet_fitting monge_fit;

    //gather points around the vertex using rings
    gather_fitting_points(v, in_points, poly_rings);

    //exit if the nb of points is too small
    if (in_points.size() < min_nb_points) {
      std::cerr << "Too few points to perform the fitting" << std::endl;
      exit(1);
    }

    //For Ridges we need at least 3rd order info
    assert(d_monge >= 3);
    // run the main fct : perform the fitting
    monge_form = monge_fit(in_points.begin(), in_points.end(),
      d_fitting, d_monge);

    //switch min-max ppal curv/dir wrt the mesh orientation
    const Vector_3 normal_mesh = compute_facets_average_unit_normal(P, v);
    monge_form.comply_wrt_given_normal(normal_mesh);

    //Store monge data needed for ridge computations in property maps
    vertex_d1_pm[v] = monge_form.maximal_principal_direction();
    vertex_d2_pm[v] = monge_form.minimal_principal_direction();
    vertex_k1_pm[v] = monge_form.coefficients()[0];
    vertex_k2_pm[v] = monge_form.coefficients()[1];
    vertex_b0_pm[v] = monge_form.coefficients()[2];
    vertex_b3_pm[v] = monge_form.coefficients()[5];
    if (d_monge >= 4) {
      //= 3*b1^2+(k1-k2)(c0-3k1^3)
      vertex_P1_pm[v] =
        3 * monge_form.coefficients()[3] * monge_form.coefficients()[3]
        + (monge_form.coefficients()[0] - monge_form.coefficients()[1])
        * (monge_form.coefficients()[6]
          -3 * monge_form.coefficients()[0] * monge_form.coefficients()[0]
          * monge_form.coefficients()[0]);
      //= 3*b2^2+(k2-k1)(c4-3k2^3)
      vertex_P2_pm[v] =
        3 * monge_form.coefficients()[4] * monge_form.coefficients()[4]
        + (-monge_form.coefficients()[0] + monge_form.coefficients()[1])
        *(monge_form.coefficients()[10]
          -3 * monge_form.coefficients()[1] * monge_form.coefficients()[1]
          * monge_form.coefficients()[1]);
    }
  }
}

