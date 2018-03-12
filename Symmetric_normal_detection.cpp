////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-02-08
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Symmetric_normal_detection.h"

#include <iostream>
#include <fstream>

#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/read_ply_points.h>

#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>

#ifdef _WIN32
#include <windows.h>
#endif
#include <gl/gl.h>
#include "Color_256.h"

#define PI 3.1415926535897932384626433832795
#define DEGENERATE_THRESHOLD 1e-4

/*!
 * \brief Symmetric_normal derives from Shape_base.
 * This shape is more restrict than Unit_normal:
 *   - z axis symmetric
 * \note Input should be unit vector and corresponding points.
 * \note symmetry may be extremely unbalanced
 */
template <class Traits>
class Symmetric_normal : public CGAL::Shape_detection_3::Shape_base<Traits> {
public:
  typedef typename Traits::FT FT;
  typedef typename Traits::Point_3 Point_3;
  typedef typename Traits::Vector_3 Vector_3;

public:
  Symmetric_normal() : CGAL::Shape_detection_3::Shape_base<Traits>() {}

  // Conversion function
  operator typename Vector_3() {
    return m_normal;
  }

  // Computes squared Euclidean distance from query point to the shape.
  virtual FT squared_distance(const Point_3 &p) const {
    // squared vector difference
    const FT d0 = CGAL::squared_distance(p, CGAL::ORIGIN + m_normal);
    const FT d1 = CGAL::squared_distance(p, CGAL::ORIGIN + m_normal_sym);
    return d0 < d1 ? d0 : d1;
  }

  // Returns a string with shape parameters.
  virtual std::string info() const {
    std::stringstream sstr;
    sstr << "Type: Symmetric_normal ("
      << m_normal.x() << ' ' << m_normal.y() << ' ' << m_normal.z()
      << ") #Pts: " << this->m_indices.size();
    return sstr.str();
  }

protected:
  // Constructs shape based on minimal set of samples from the input data.
  virtual void create_shape(const std::vector<std::size_t> &indices) {
    m_normal = this->normal(indices[0]) + this->normal(indices[1]) + this->normal(indices[2]);
    if (m_normal.squared_length() < FT(DEGENERATE_THRESHOLD))
      return;

    m_normal /= CGAL::sqrt(m_normal.squared_length());
    //check deviation of the 3 normal
    for (std::size_t i = 0; i < 3; ++i)
      if (m_normal * this->normal(indices[i]) < this->m_normal_threshold)
        return;

    m_normal_sym = Vector_3(-m_normal.x(), -m_normal.y(), m_normal.z());

    this->m_is_valid = true;
  }

  // Computes squared Euclidean distance from a set of points.
  virtual void squared_distance(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &dists) const {
    for (std::size_t i = 0; i < indices.size(); i++) {
      // squared vector difference
      const FT d0 = CGAL::squared_distance(this->point(indices[i]), CGAL::ORIGIN + m_normal);
      const FT d1 = CGAL::squared_distance(this->point(indices[i]), CGAL::ORIGIN + m_normal_sym);
      dists[i] = d0 < d1 ? d0 : d1;
    }
  }

  // Computes the normal deviation between shape and a set of points with normals.
  virtual void cos_to_normal(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &angles) const {
    for (std::size_t i = 0; i < indices.size(); i++) {
      const FT c0 = this->normal(indices[i]) * m_normal;
      const FT c1 = this->normal(indices[i]) * m_normal_sym;
      angles[i] = c0 > c1 ? c0 : c1;
    }
  }

  // Returns the number of required samples for construction.
  virtual std::size_t minimum_sample_size() const {
    return 3;
  }

  // no connected component concept in this shape
  virtual std::size_t connected_component(std::vector<std::size_t>& indices, FT cluster_epsilon) {
    return indices.size();
  }

private:
  // normal and its symmetric counterpart
  Vector_3 m_normal;
  Vector_3 m_normal_sym;
};

// in layer cone angle, 10 degree
#define IN_CONE_ANGLE (PI / 18.0)
// out layer cone angle, 45 degree
// #define OUT_CONE_ANGLE (PI / 4.0)
// out layer cone angle, 60 degree
#define OUT_CONE_ANGLE (PI / 3.0)

/*!
 * \brief Constrained_symmetric_normal derives from Shape_base.
 * This shape is more restrict than Symmetric_normal:
 *   - z axis symmetric
 *   - normal close to z axis and too far away are excluded
 *   - normals in a cone with up right z axis
 *   - the projection on xy plane of the normal is restricted to the orthogonal facade direction
 * Here data is facade unit orthogonal directions on xy plane
 * \note symmetry may be extremely unbalanced
 * \note Input should be unit vector pointing upwards and corresponding points.
 */
template <class Traits>
class Constrained_symmetric_normal :
  public CGAL::Shape_detection_3::Shape_base<Traits> {
public:
  typedef typename Traits::FT FT;
  typedef typename Traits::Point_3 Point_3;
  typedef typename Traits::Vector_3 Vector_3;

static std::vector<Vector_3> *s_data;

public:
  Constrained_symmetric_normal() : CGAL::Shape_detection_3::Shape_base<Traits>() {}

  // Conversion function
  operator typename Vector_3() {
    return m_normal;
  }

  // Computes squared Euclidean distance from query point to the shape.
  virtual FT squared_distance(const Point_3 &p) const {
    // squared vector difference
    const FT d0 = CGAL::squared_distance(p, CGAL::ORIGIN + m_normal);
    const FT d1 = CGAL::squared_distance(p, CGAL::ORIGIN + m_normal_sym);
    return d0 < d1 ? d0 : d1;
  }

  // Returns a string with shape parameters.
  virtual std::string info() const {
    std::stringstream sstr;
    sstr << "Type: Constrained_symmetric_normal ("
      << m_normal.x() << ' ' << m_normal.y() << ' ' << m_normal.z()
      << ") #Pts: " << this->m_indices.size();
    return sstr.str();
  }

protected:
  // Constructs shape based on minimal set of samples from the input data.
  virtual void create_shape(const std::vector<std::size_t> &indices) {
    m_normal = this->normal(indices[0]) + this->normal(indices[1]) + this->normal(indices[2]);
    if (m_normal.squared_length() < FT(DEGENERATE_THRESHOLD))
      return;

    m_normal /= CGAL::sqrt(m_normal.squared_length());
    //check deviation of the 3 normal
    for (std::size_t i = 0; i < 3; ++i)
      if (m_normal * this->normal(indices[i]) < this->m_normal_threshold)
        return;

    // check in the cone
    if (m_normal.z() > FT(std::cos(IN_CONE_ANGLE)) || m_normal.z() < FT(std::cos(OUT_CONE_ANGLE)))
      return;

    // choose one of the orthogonal directions to snap to
    Vector_3 proj_xy(m_normal.x(), m_normal.y(), FT(0.0));
    proj_xy /= CGAL::sqrt(proj_xy.squared_length());
    Vector_3 snap_to = s_data->front();
    FT max_sim(-1.0);
    for (const auto &cd : *s_data) {
      if (CGAL::abs(proj_xy * cd) > max_sim) {
        max_sim = CGAL::abs(proj_xy * cd);
        snap_to = cd;
      }
    }
    if (max_sim < this->m_normal_threshold)
      return;

    // snap to
    const FT ratio = CGAL::sqrt(FT(1.0) - m_normal.z() * m_normal.z());
    m_normal = Vector_3(snap_to.x() * ratio, snap_to.y() * ratio, m_normal.z());

    m_normal_sym = Vector_3(-m_normal.x(), -m_normal.y(), m_normal.z());

    this->m_is_valid = true;
  }

  // Computes squared Euclidean distance from a set of points.
  virtual void squared_distance(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &dists) const {
    for (std::size_t i = 0; i < indices.size(); i++) {
      // squared vector difference
      const FT d0 = CGAL::squared_distance(this->point(indices[i]), CGAL::ORIGIN + m_normal);
      const FT d1 = CGAL::squared_distance(this->point(indices[i]), CGAL::ORIGIN + m_normal_sym);
      dists[i] = d0 < d1 ? d0 : d1;
    }
  }

  // Computes the normal deviation between shape and a set of points with normals.
  virtual void cos_to_normal(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &angles) const {
    for (std::size_t i = 0; i < indices.size(); i++) {
      const FT c0 = this->normal(indices[i]) * m_normal;
      const FT c1 = this->normal(indices[i]) * m_normal_sym;
      angles[i] = c0 > c1 ? c0 : c1;
    }
  }

  // Returns the number of required samples for construction.
  virtual std::size_t minimum_sample_size() const {
    return 3;
  }

  // no connected component concept in this shape
  virtual std::size_t connected_component(std::vector<std::size_t>& indices, FT cluster_epsilon) {
    return indices.size();
  }

private:
  // normal and its symmetric counterpart
  Vector_3 m_normal;
  Vector_3 m_normal_sym;
};

template <typename Traits>
std::vector<Kernel2::Vector_3> *Constrained_symmetric_normal<Traits>::s_data = nullptr;

namespace Algs {


void Symmetric_normal_detection::detect(
  const std::string &fname,
  const Params::Shape_detection &params,
  const bool is_constrained)
{
  typedef CGAL::Nth_of_tuple_property_map<0, Point_with_normal_point> Point_map;
  typedef CGAL::Nth_of_tuple_property_map<1, Point_with_normal_point> Normal_map;
  typedef CGAL::Nth_of_tuple_property_map<2, Point_with_normal_point> Normal_point_map;
  // In Shape_detection_traits the basic types, i.e., Point and Vector types
  // as well as iterator type and property maps, are defined.
  typedef CGAL::Shape_detection_3::Shape_detection_traits
    <Kernel2, Pwnp_vector, Normal_point_map, Normal_map> Traits;
  typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits> Efficient_ransac;

  typedef Symmetric_normal<Traits> Symmetric_normal;
  typedef Constrained_symmetric_normal<Traits> Constrained_symmetric_normal;

  m_points.clear();
  // Loads point set from a file.
  std::ifstream ifs(fname);
  if (!ifs.is_open())
    return;
  bool is_successful = false;
  if (fname.substr(fname.find_last_of('.')) == ".pwn")
    is_successful = CGAL::read_xyz_points_and_normals(
      ifs, std::back_inserter(m_points), Point_map(), Normal_map());
  else if (fname.substr(fname.find_last_of('.')) == ".ply")
    is_successful = CGAL::read_ply_points_and_normals(
      ifs, std::back_inserter(m_points), Point_map(), Normal_map());
  ifs.close();
  if (!is_successful) {
    std::cerr << "Error: cannot read file " << fname << std::endl;
    return;
  }

  // update viewing bbox
  if (!m_points.empty()) {
    m_bbox = m_points.front().get<0>().bbox();
    for (auto &p : m_points)
      m_bbox = m_bbox + p.get<0>().bbox();
  }

  // setup normal to points on sphere
  for (auto &pwnp : m_points)
    pwnp.get<2>() = CGAL::ORIGIN + pwnp.get<1>();

  Efficient_ransac::Parameters parameters;
  // Sets probability to miss the largest primitive at each iteration.
  parameters.probability = params.probability;
  // Detect shapes with at least 500 points.
  parameters.min_points = params.min_points;
  // Sets maximum Euclidean distance between a point and a shape.
  parameters.epsilon = params.epsilon;
  // Sets maximum Euclidean distance between points to be clustered.
  parameters.cluster_epsilon = params.cluster_epsilon;
  // Sets maximum normal deviation.
  // 0.9 < dot(surface_normal, point_normal); 
  parameters.normal_threshold = params.normal_threshold;

  std::cout << "Shape detection...";

  Efficient_ransac ransac;
  ransac.set_input(m_points);
  if (is_constrained) {
    std::vector<Kernel2::Vector_3> facade_directions;
    // facade_directions.push_back({1.0, 0.0, 0.0});
    // facade_directions.push_back({0.0, 1.0, 0.0});

    // test2
    const double angle = 81.0 / 180.0 * PI;
    facade_directions.push_back({std::cos(angle), std::sin(angle), 0});
    facade_directions.push_back({std::cos(angle + PI / 2.0), std::sin(angle + PI / 2.0), 0});

    Constrained_symmetric_normal::s_data = &facade_directions;
    ransac.add_shape_factory<Constrained_symmetric_normal>();
  }
  else
    ransac.add_shape_factory<Symmetric_normal>();
  ransac.preprocess();
  ransac.detect(parameters);

  Efficient_ransac::Shape_range shapes = ransac.shapes();
  FT coverage = FT(m_points.size() - ransac.number_of_unassigned_points()) / FT(m_points.size());
  // Prints number of assigned shapes and unassigned points.
  std::cout << shapes.size() << " primitives, "
    << coverage << " coverage" << std::endl;
  for (const auto &s : shapes)
    std::cout << s->info() << std::endl;

  m_point_shapes = std::vector<int>(m_points.size(), -1);
  int sidx = 0;
  for (const auto s : shapes) {
    FT sum_distances = 0;
    for (const std::size_t pidx : s->indices_of_assigned_points()) {
      sum_distances += CGAL::sqrt(s->squared_distance(m_points[pidx].get<2>()));
      m_point_shapes[pidx] = sidx;
    }

    FT average_distance = sum_distances / s->indices_of_assigned_points().size();
    std::cout << " average distance: " << average_distance << std::endl;

    ++sidx;
  }
  std::cout << "done" << std::endl;

  // random color table
  m_shape_colors = std::vector<std::size_t>(shapes.size(), 0);
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  for (std::size_t &c : m_shape_colors)
    c = static_cast<std::size_t>(std::rand() % 255);
}

void Symmetric_normal_detection::draw()
{
  if (m_points.empty())
    return;

  // draw point cloud with respect color
  // ::glDisable(GL_LIGHTING);
  ::glEnable(GL_LIGHTING);
  ::glPointSize(2.0);
  ::glBegin(GL_POINTS);
  for (std::size_t pidx = 0; pidx < m_points.size(); ++pidx) {
    if (m_point_shapes[pidx] >= 0) {
      const std::size_t cidx = m_shape_colors[m_point_shapes[pidx]];
      ::glColor3ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx));
    }
    else
      ::glColor3ub(192, 192, 192);

    const Kernel2::Vector_3 &n = m_points[pidx].get<1>();
    ::glNormal3d(n.x(), n.y(), n.z());
    const Kernel2::Point_3 &p = m_points[pidx].get<0>();
    ::glVertex3d(p.x(), p.y(), p.z());
  }
  ::glEnd();

  const Kernel2::Point_3 sphere_center(
    (m_bbox.xmin() + m_bbox.xmax()) / 2.0,
    (m_bbox.ymin() + m_bbox.ymax()) / 2.0,
    (m_bbox.zmax() - m_bbox.zmin()) / 2.0 + m_bbox.zmax());

  // draw normals at bbox center
  // ::glDisable(GL_LIGHTING);
  ::glEnable(GL_LIGHTING);
  ::glPointSize(2.0);
  ::glBegin(GL_POINTS);
  for (std::size_t pidx = 0; pidx < m_points.size(); ++pidx) {
    if (m_point_shapes[pidx] >= 0) {
      const std::size_t cidx = m_shape_colors[m_point_shapes[pidx]];
      ::glColor3ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx));
    }
    else
      ::glColor3ub(192, 192, 192);

    const Kernel2::Vector_3 &n = m_points[pidx].get<1>();
    ::glNormal3d(n.x(), n.y(), n.z());
    const Kernel2::Vector_3 &p = m_points[pidx].get<1>();
    ::glVertex3d(sphere_center.x() + p.x(), sphere_center.y() + p.y(), sphere_center.z() + p.z());
  }
  ::glEnd();
}

} // Algs

#undef PI // PI
