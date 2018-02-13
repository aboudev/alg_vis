////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-30
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Horizontal_plane_detection.h"

#include <iostream>
#include <fstream>

#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/read_ply_points.h>

#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/regularize_planes.h>
#include <CGAL/convex_hull_2.h>

#ifdef _WIN32
#include <windows.h>
#endif
#include <gl/gl.h>
#include "Color_256.h"

#define DEGENERATE_THRESHOLD 1e-4

/*!
 * \brief Horizontal_plane derives from Shape_base.
 * The plane is represented by its normal vector and distance to the origin.
 */
/*!
 * \brief Horizontal_plane derives from Shape_base.
 * The plane is guaranteed to be horizontal.
 */
template <class Traits>
class Horizontal_plane : public CGAL::Shape_detection_3::Shape_base<Traits> {
public:
  typedef typename Traits::FT FT;
  typedef typename Traits::Point_3 Point_3;
  typedef typename Traits::Vector_3 Vector_3;

public:
  Horizontal_plane() : CGAL::Shape_detection_3::Shape_base<Traits>() {}

  // Conversion function
  operator typename Traits::Plane_3() {
    return Traits::Plane_3(m_point_on_primitive, m_normal);
  }

  // Computes squared Euclidean distance from query point to the shape.
  virtual FT squared_distance(const Point_3 &p) const {
    const FT sd = (p - m_point_on_primitive) * m_normal;
    return sd * sd;
  }

  // Returns a string with shape parameters.
  virtual std::string info() const {
    std::stringstream sstr;
    sstr << "Type: Horizontal_plane (" <<
      m_normal.x() << ' '
      << m_normal.y() << ' '
      << m_normal.z() << ")x - "
      << (m_point_on_primitive - CGAL::ORIGIN) * m_normal << " = 0"
      << " #Pts: " << this->m_indices.size();
    return sstr.str();
  }

protected:
  // Constructs shape based on minimal set of samples from the input data.    
  virtual void create_shape(const std::vector<std::size_t> &indices) {
    const Point_3 p0 = this->point(indices[0]);
    const Point_3 p1 = this->point(indices[1]);
    const Point_3 p2 = this->point(indices[2]);

    m_normal = CGAL::cross_product(p1 - p0, p2 - p0);

    const FT length = CGAL::sqrt(m_normal.squared_length());
    // Are the points almost singular?
    if (length < FT(DEGENERATE_THRESHOLD))
      return;

    m_normal /= length;
    // check deviation of the 3 normal
    for (std::size_t i = 0; i < 3; ++i)
      if (this->normal(indices[i]) * m_normal < this->m_normal_threshold)
        return;

    // check snap to vertical
    if (m_normal.z() < this->m_normal_threshold)
      return;
    m_normal = Vector_3(0.0, 0.0, 1.0);

    m_point_on_primitive = CGAL::ORIGIN + ((p0 - CGAL::ORIGIN)
      + (p1 - CGAL::ORIGIN)
      + (p2 - CGAL::ORIGIN)) / FT(3.0);

    m_base1 = CGAL::cross_product(p1 - p0, m_normal);
    m_base1 /= CGAL::sqrt(m_base1.squared_length());

    m_base2 = CGAL::cross_product(m_base1, m_normal);
    m_base2 /= CGAL::sqrt(m_base2.squared_length());

    this->m_is_valid = true;
  }

  // Computes squared Euclidean distance from a set of points.
  virtual void squared_distance(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &dists) const {
    for (std::size_t i = 0; i < indices.size(); i++) {
      const FT sd = (this->point(indices[i]) - m_point_on_primitive) * m_normal;
      dists[i] = sd * sd;
    }
  }

  // Computes the normal deviation between shape and a set of points with normals.
  // signed cosine, not absolute
  virtual void cos_to_normal(
    const std::vector<std::size_t> &indices,
    std::vector<FT> &angles) const {
    for (std::size_t i = 0; i < indices.size(); i++)
      angles[i] = this->normal(indices[i]) * m_normal;
  }

  // Returns the number of required samples for construction.
  virtual std::size_t minimum_sample_size() const {
    return 3;
  }

  virtual void parameters(
    const std::vector<std::size_t> &indices,
    std::vector<std::pair<FT, FT> > &parameterSpace,
    FT &, FT min[2], FT max[2]) const {
    // Transform first point before to initialize min/max
    Vector_3 p = m_point_on_primitive - this->point(indices[0]);
    FT u = p * m_base1;
    FT v = p * m_base2;
    parameterSpace[0] = std::pair<FT, FT>(u, v);
    min[0] = max[0] = u;
    min[1] = max[1] = v;

    for (std::size_t i = 1; i < indices.size(); ++i) {
      p = m_point_on_primitive - this->point(indices[i]);
      u = p * m_base1;
      v = p * m_base2;
      min[0] = (CGAL::min)(min[0], u);
      max[0] = (CGAL::max)(max[0], u);
      min[1] = (CGAL::min)(min[1], v);
      max[1] = (CGAL::max)(max[1], v);
      parameterSpace[i] = std::pair<FT, FT>(u, v);
    }
  }

  // if the shape is developable and have parameters function
  // set this function to return true to use the internal
  // planar parameterization to detect the connected component
  virtual bool supports_connected_component() const {
    return true;
  };

private:
  Point_3 m_point_on_primitive;
  Vector_3 m_normal;

  // plane base for parameterization
  Vector_3 m_base1, m_base2;
};

namespace Algs {

void Horizontal_plane_detection::detect(
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
  typedef Horizontal_plane<Traits> Horizontal_plane;

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
  ransac.add_shape_factory<Horizontal_plane>();
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

  // plane regularization
  //Efficient_ransac::Plane_range planes = ransac.planes();
  //CGAL::regularize_planes(m_points,
  //  Point_map(),
  //  planes,
  //  CGAL::Shape_detection_3::Plane_map<Traits>(),
  //  CGAL::Shape_detection_3::Point_to_shape_index_map<Traits>(m_points, planes),
  //  true, //Regularize parallelism
  //  true, // Regularize orthogonality
  //  false, // Do not regularize coplanarity
  //  true, // Regularize Z-symmetry (default)
  //  10); // 10 degrees of tolerance for parallelism / orthogonality

  std::cout << "done" << std::endl;

  if (!m_points.empty()) {
    m_bbox = m_points.begin()->first.bbox();
    for (auto &p : m_points)
      m_bbox = m_bbox + p.first.bbox();
  }

  // rendering data
  m_convex_hulls.clear();
  for (const auto s : shapes) {
    std::list<Kernel2::Point_3> pts;
    for (const std::size_t &pidx : s->indices_of_assigned_points())
      pts.push_back(m_points[pidx].first);

    const Kernel2::Plane_3 plane = static_cast<Kernel2::Plane_3>(
      *dynamic_cast<Horizontal_plane *>(s.get()));
    const Kernel2::Point_3 origin = plane.projection(pts.front());

    Kernel2::Vector_3 base1 = plane.base1();
    Kernel2::Vector_3 base2 = plane.base2();
    base1 = base1 / std::sqrt(base1.squared_length());
    base2 = base2 / std::sqrt(base2.squared_length());

    Kernel2::Line_3 baseLine1(origin, base1);
    Kernel2::Line_3 baseLine2(origin, base2);

    std::vector<Kernel2::Point_2> vec2DCoord;
    for (const auto &p : pts) {
        const Kernel2::Point_3 point = plane.projection(p);
        Kernel2::Vector_3 xVector(origin, baseLine1.projection(point));
        Kernel2::Vector_3 yVector(origin, baseLine2.projection(point));
        double x = std::sqrt(xVector.squared_length());
        double y = std::sqrt(yVector.squared_length());
        x = xVector * base1 < 0 ? -x : x;
        y = yVector * base2 < 0 ? -y : y;
        vec2DCoord.push_back(Kernel2::Point_2(x, y));
    }

    std::vector<Kernel2::Point_2> cvx_hull_2;
    CGAL::convex_hull_2(vec2DCoord.begin(), vec2DCoord.end(), std::back_inserter(cvx_hull_2));

    std::vector<Kernel2::Point_3> cvx_hull_3;
    for (const auto &p : cvx_hull_2)
      cvx_hull_3.push_back(origin + p.x() * base1 + p.y() * base2);

    m_convex_hulls.push_back(cvx_hull_3);
  }

  // random color table
  m_shape_colors = std::vector<std::size_t>(m_convex_hulls.size(), 0);
  std::srand(static_cast<unsigned int>(std::time(nullptr)));
  for (std::size_t &c : m_shape_colors)
    c = static_cast<std::size_t>(std::rand() % 255);
}

void Horizontal_plane_detection::draw()
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

  // draw convex hull of shape points
  ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  ::glEnable(GL_BLEND);
  ::glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (std::size_t sidx = 0; sidx < m_convex_hulls.size(); ++sidx) {
    const auto &cvh = m_convex_hulls[sidx];
    const std::size_t cidx = m_shape_colors[sidx];
    ::glColor4ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx), 150);
    ::glBegin(GL_POLYGON);
    // ::glBegin(GL_LINE_LOOP);
    for (const auto &p : cvh)
      ::glVertex3d(p.x(), p.y(), p.z());
    ::glEnd();
  }
}

} // Algs
