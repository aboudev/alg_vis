////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-02-01
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Unit_normal_detection.h"

#include <iostream>
#include <fstream>

#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/read_ply_points.h>

#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/convex_hull_2.h>

#ifdef _WIN32
#include <windows.h>
#endif
#include <gl/gl.h>
#include "Color_256.h"

/*!
 * \brief Unit_normal derives from Shape_base.
 * Only calculate normal deviation.
 */
template <class Traits>
class Unit_normal : public CGAL::Shape_detection_3::Shape_base<Traits> {
public:
  typedef typename Traits::FT FT;
  typedef typename Traits::Point_3 Point;
  typedef typename Traits::Vector_3 Vector;

public:
  Unit_normal() : CGAL::Shape_detection_3::Shape_base<Traits>() {}

  // Conversion function
  operator typename Vector() {
    return m_normal;
  }

  // Computes squared Euclidean distance from query point to the shape.
  virtual FT squared_distance(const Point &p) const {
    return FT(0.0);
  }

  // Returns a string with shape parameters.
  virtual std::string info() const {
    std::stringstream sstr;
    sstr << "Type: unit normal ("
      << m_normal.x() << ' ' << m_normal.y() << ' ' << m_normal.z()
      << ") #Pts: " << this->m_indices.size();
    return sstr.str();
  }

protected:
  // Constructs shape based on minimal set of samples from the input data.    
  virtual void create_shape(const std::vector<std::size_t> &indices) {
    m_normal = this->normal(indices[0]);
    this->m_is_valid = true;
  }

  // Computes squared Euclidean distance from a set of points.
  virtual void squared_distance(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &dists) const {
    for (std::size_t i = 0; i < indices.size(); i++)
      dists[i] = FT(0.0);
  }

  // Computes the normal deviation between shape and a set of points with normals.
  virtual void cos_to_normal(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &angles) const {
    for (std::size_t i = 0; i < indices.size(); i++)
      angles[i] = CGAL::abs(this->normal(indices[i]) * m_normal);
  }

  // Returns the number of required samples for construction.
  virtual std::size_t minimum_sample_size() const {
    return 1;
  }

private:
  Vector m_normal;
};

namespace Algs {

void Unit_normal_detection::detect(
  const std::string &fname,
  const Params::Shape_detection &params)
{
  typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
  typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
  // In Shape_detection_traits the basic types, i.e., Point and Vector types
  // as well as iterator type and property maps, are defined.
  typedef CGAL::Shape_detection_3::Shape_detection_traits
    <Kernel2, Pwn_vector, Point_map, Normal_map> Traits;
  typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits> Efficient_ransac;
  typedef Unit_normal<Traits> Unit_normal;

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
  ransac.add_shape_factory<Unit_normal>();
  ransac.preprocess();
  ransac.detect(parameters);

  Efficient_ransac::Shape_range shapes = ransac.shapes();
  FT coverage = FT(m_points.size() - ransac.number_of_unassigned_points()) / FT(m_points.size());
  // Prints number of assigned shapes and unassigned points.
  std::cout << shapes.size() << " primitives, "
    << coverage << " coverage" << std::endl;

  m_point_shapes = std::vector<int>(m_points.size(), -1);
  int sidx = 0;
  for (const auto s : shapes) {
    FT sum_distances = 0;
    for (const std::size_t pidx : s->indices_of_assigned_points()) {
      sum_distances += CGAL::sqrt(s->squared_distance(m_points[pidx].first));
      m_point_shapes[pidx] = sidx;
    }

    FT average_distance = sum_distances / s->indices_of_assigned_points().size();
    std::cout << " average distance: " << average_distance << std::endl;

    ++sidx;
  }
  std::cout << "done" << std::endl;

  // update viewing bbox
  if (!m_points.empty()) {
    m_bbox = m_points.begin()->first.bbox();
    for (auto &p : m_points)
      m_bbox = m_bbox + p.first.bbox();
  }

  // random color table
  m_shape_colors = std::vector<std::size_t>(shapes.size(), 0);
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  for (std::size_t &c : m_shape_colors)
    c = static_cast<std::size_t>(std::rand() % 255);
}

void Unit_normal_detection::draw()
{
  if (m_points.empty())
    return;

  // draw point cloud with respect color
  ::glDisable(GL_LIGHTING);
  ::glPointSize(5.0);
  ::glBegin(GL_POINTS);
  for (std::size_t pidx = 0; pidx < m_points.size(); ++pidx) {
    if (m_point_shapes[pidx] >= 0) {
      const std::size_t cidx = m_shape_colors[m_point_shapes[pidx]];
      ::glColor3ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx));
    }
    else
      ::glColor3ub(0, 0, 0);

    const Kernel2::Point_3 &p = m_points[pidx].first;
    ::glVertex3d(p.x(), p.y(), p.z());
  }
  ::glEnd();

  const Kernel2::Point_3 bbx_center(
    (m_bbox.xmin() + m_bbox.xmax()) / 2.0,
    (m_bbox.ymin() + m_bbox.ymax()) / 2.0,
    (m_bbox.zmin() + m_bbox.zmax()) / 2.0);

  // draw normals at bbox center
  ::glDisable(GL_LIGHTING);
  ::glPointSize(5.0);
  ::glBegin(GL_POINTS);
  for (std::size_t pidx = 0; pidx < m_points.size(); ++pidx) {
    if (m_point_shapes[pidx] >= 0) {
      const std::size_t cidx = m_shape_colors[m_point_shapes[pidx]];
      ::glColor3ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx));
    }
    else
      ::glColor3ub(0, 0, 0);

    const Kernel2::Vector_3 &n = m_points[pidx].second;
    ::glVertex3d(bbx_center.x() + n.x(), bbx_center.y() + n.y(), bbx_center.z() + n.z());
  }
  ::glEnd();

  // draw detected nomrals at bbox center
  // ::glDisable(GL_LIGHTING);
  // ::glLineWidth(3.0);
  // ::glBegin(GL_LINES);
  // for (std::size_t pidx = 0; pidx < m_points.size(); ++pidx) {
  //   if (m_point_shapes[pidx] >= 0) {
  //     const std::size_t cidx = m_shape_colors[m_point_shapes[pidx]];
  //     ::glColor3ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx));
  //   }
  //   else
  //     ::glColor3ub(0, 0, 0);

  //   const Kernel2::Vector_3 &n = m_points[pidx].second;
  //   ::glVertex3d(bbx_center.x() + n.x(), bbx_center.y() + n.y(), bbx_center.z() + n.z());
  // }
  // ::glEnd();
}

} // Algs
