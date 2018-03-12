////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Shape_detection.h"

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

namespace Algs {

void Shape_detection::detect(const std::string &fname, const Params::Shape_detection &params)
{
  typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
  typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
  // In Shape_detection_traits the basic types, i.e., Point and Vector types
  // as well as iterator type and property maps, are defined.
  typedef CGAL::Shape_detection_3::Shape_detection_traits
    <Kernel2, Pwn_vector, Point_map, Normal_map>               Traits;
  typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
  typedef CGAL::Shape_detection_3::Plane<Traits>               RansacPlane;

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
  ransac.add_shape_factory<RansacPlane>();
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
  Efficient_ransac::Plane_range planes = ransac.planes();
  CGAL::regularize_planes(m_points,
    Point_map(),
    planes,
    CGAL::Shape_detection_3::Plane_map<Traits>(),
    CGAL::Shape_detection_3::Point_to_shape_index_map<Traits>(m_points, planes),
    true, //Regularize parallelism
    true, // Regularize orthogonality
    false, // Do not regularize coplanarity
    true, // Regularize Z-symmetry (default)
    10); // 10 degrees of tolerance for parallelism / orthogonality

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
      *dynamic_cast<RansacPlane *>(s.get()));
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

void Shape_detection::draw()
{
  if (m_points.empty())
    return;

  // draw point cloud with respect color
  // ::glDisable(GL_LIGHTING);
  // ::glEnable(GL_LIGHTING);
  // ::glPointSize(3.0);
  // ::glBegin(GL_POINTS);
  // for (std::size_t pidx = 0; pidx < m_points.size(); ++pidx) {
  //   if (m_point_shapes[pidx] >= 0) {
  //     const std::size_t cidx = m_shape_colors[m_point_shapes[pidx]];
  //     ::glColor3ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx));
  //   }
  //   else
  //     ::glColor3ub(192, 192, 192);

  //   const Kernel2::Vector_3 &n = m_points[pidx].second;
  //   ::glNormal3d(n.x(), n.y(), n.z());
  //   const Kernel2::Point_3 &p = m_points[pidx].first;
  //   ::glVertex3d(p.x(), p.y(), p.z());
  // }
  // ::glEnd();

  // draw convex hull of shape points
  ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  ::glEnable(GL_BLEND);
  ::glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (std::size_t sidx = 0; sidx < m_convex_hulls.size(); ++sidx) {
    const auto &cvh = m_convex_hulls[sidx];
    const std::size_t cidx = m_shape_colors[sidx];
    ::glColor4ub(Color_256::r(cidx), Color_256::g(cidx), Color_256::b(cidx), 180);
    ::glBegin(GL_POLYGON);
    // ::glBegin(GL_LINE_LOOP);
    for (const auto &p : cvh)
      ::glVertex3d(p.x(), p.y(), p.z());
    ::glEnd();
  }
}

} // Algs
