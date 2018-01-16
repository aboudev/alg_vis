////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#include "Shape_detection.h"

#include <iostream>
#include <fstream>

#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/regularize_planes.h>
#include <CGAL/Timer.h>

// slicer
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/AABB_halfedge_graph_segment_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polygon_mesh_slicer.h>

typedef CGAL::Polyhedron_3<Kernel2> Epic_Polyhedron;
typedef std::vector<Kernel2::Point_3> Polyline_type;
typedef std::list<Polyline_type> Polylines;
typedef CGAL::AABB_halfedge_graph_segment_primitive<Epic_Polyhedron> HGSP;
typedef CGAL::AABB_traits<Kernel2, HGSP> AABB_traits;
typedef CGAL::AABB_tree<AABB_traits> AABB_tree;

#include "Color_256.h"

namespace Algs {

Shape_detection::Shape_detection() :
  m_dm(m_ransac),
  // no default constructor for shape_range, although invalid
  m_bestshapes(m_ransac.shapes()) {}

void Shape_detection::detect(const std::string &fname)
{
  std::cout << "Shape detection...";

  m_points.clear();
  // Loads point set from a file. 
  // read_xyz_points_and_normals takes an OutputIterator for storing the points
  // and a property map to store the normal vector with each point.
  std::ifstream stream(fname);
  if (!stream ||
    !CGAL::read_xyz_points_and_normals(stream,
      std::back_inserter(m_points),
      Point_map(),
      Normal_map())) {
    std::cerr << "Error: cannot read file cube.pwn" << std::endl;
    return;
  }

  // Instantiates shape detection engine.
  // Provides the input data.
  m_ransac.set_input(m_points);
  // Registers planar shapes via template method.
  m_ransac.add_shape_factory<RansacPlane>();
  //// Detects registered shapes with default parameters.
  //m_ransac.detect();
  //// Prints number of detected shapes.
  //std::cout << m_ransac.shapes().end() - m_ransac.shapes().begin() << " shapes detected." << std::endl;

  // Measures time before setting up the shape detection.
  CGAL::Timer time;
  time.start();
  // Build internal data structures.
  m_ransac.preprocess();
  // Measures time after preprocessing.
  time.stop();
  std::cout << "preprocessing took: " << time.time() * 1000 << "ms" << std::endl;
  // Perform detection several times and choose result with highest coverage.
  Efficient_ransac::Shape_range shapes = m_ransac.shapes();
  FT best_coverage = 0;
  for (size_t i = 0; i < 6; i++) {
    // Reset timer.
    time.reset();
    time.start();
    // Detects shapes.
    m_ransac.detect();
    // Measures time after detection.
    time.stop();
    // Compute coverage, i.e. ratio of the points assigned to a shape.
    FT coverage = FT(m_points.size() - m_ransac.number_of_unassigned_points())
      / FT(m_points.size());
    // Prints number of assigned shapes and unsassigned points.
    std::cout << "time: " << time.time() * 1000 << "ms" << std::endl;
    std::cout << m_ransac.shapes().end() - m_ransac.shapes().begin() << " primitives, "
      << coverage << " coverage" << std::endl;

    std::size_t cnt = 0;
    for (auto &s : m_ransac.shapes()) {
      //std::cout << s->info() << std::endl;
      //std::cout << s->indices_of_assigned_points().size() << std::endl;
      cnt += s->indices_of_assigned_points().size();
    }
    std::cout << m_points.size() << " = " << cnt << " + " << m_ransac.number_of_unassigned_points() << std::endl;

    // Choose result with highest coverage.
    if (coverage > best_coverage) {
      best_coverage = coverage;
      // Efficient_ransac::shapes() provides
      // an iterator range to the detected shapes. 
      shapes = m_ransac.shapes();
      m_bestshapes = shapes;
    }
  }

  Efficient_ransac::Shape_range::iterator it = shapes.begin();
  while (it != shapes.end()) {
    boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
    // Using Shape_base::info() for printing 
    // the parameters of the detected shape.
    std::cout << (*it)->info();
    // Sums distances of points to detected shapes.
    FT sum_distances = 0;
    // Iterates through point indices assigned to each detected shape.
    std::vector<std::size_t>::const_iterator
      index_it = (*it)->indices_of_assigned_points().begin();
    while (index_it != (*it)->indices_of_assigned_points().end()) {

      // Retrieves point
      const Point_with_normal &p = *(m_points.begin() + (*index_it));
      // Adds Euclidean distance between point and shape.
      sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
      // Proceeds with next point.
      index_it++;
    }
    // Computes and prints average distance.
    FT average_distance = sum_distances / shape->indices_of_assigned_points().size();
    std::cout << " average distance: " << average_distance << std::endl;
    // Proceeds with next detected shape.
    it++;
  }

  // plane regularization in CGAL 4.9
  // described in "LOD Generation for Urban Scenes", Verdie. et al
  Efficient_ransac::Plane_range planes = m_ransac.planes();
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

  // m_view_shapes = true;

  std::cout << m_bbox << std::endl;
  if (!m_points.empty()) {
    m_bbox = m_points.begin()->first.bbox();
    for (auto &p : m_points)
      m_bbox = m_bbox + p.first.bbox();
  }
  std::cout << m_bbox << std::endl;

  return;
}

void Shape_detection::draw() {
  if (m_points.empty())
    return;

  Efficient_ransac::Shape_range shapes = m_bestshapes;
  Efficient_ransac::Shape_range::iterator it = shapes.begin();

  // shape color table
  std::vector<int> color_tab;
  for (int i = 0; it != shapes.end(); ++it, ++i) {
    double R = ((double)i / double(shapes.size())) * (double)255.0;
    int indiceLut = std::floor(R);
    color_tab.push_back(indiceLut);
  }
  std::vector<int>::iterator color_itr = color_tab.begin();

  std::set<std::size_t> pt_idx_set;
  for (std::size_t i = 0; i < m_points.size(); ++i)
    pt_idx_set.insert(i);

  ::glDisable(GL_LIGHTING);

  // draw point cloud with respect color
  ::glPointSize(5.0);
  it = shapes.begin();
  while (it != shapes.end()) {
    ::glColor3ub(Color_256::r(*color_itr), Color_256::g(*color_itr), Color_256::b(*color_itr));
    ++color_itr;
    ::glBegin(GL_POINTS);
    // Iterates through point indices assigned to each detected shape.
    std::vector<std::size_t>::const_iterator
      index_it = (*it)->indices_of_assigned_points().begin();
    while (index_it != (*it)->indices_of_assigned_points().end()) {
      // Retrieves point
      const Point_with_normal &p = *(m_points.begin() + (*index_it));
      ::glVertex3d(p.first.x(), p.first.y(), p.first.z());
      pt_idx_set.erase(*index_it);
      // Proceeds with next point.
      index_it++;
    }
    // Proceeds with next detected shape.
    it++;
    ::glEnd();
  }
  // remaining points
  std::cout << m_points.size() << " --- " << pt_idx_set.size() << std::endl;
  ::glColor3f(0.0, 0.0, 0.0);
  ::glBegin(GL_POINTS);
  for (std::set<std::size_t>::iterator itr = pt_idx_set.begin();
    itr != pt_idx_set.end(); ++itr)
    ::glVertex3d(m_points[*itr].first.x(), m_points[*itr].first.y(), m_points[*itr].first.z());
  ::glEnd();

  //::glLineWidth(1.0);
  //it = shapes.begin();
  //color_itr = color_tab.begin();
  //while (it != shapes.end()) {
  //    ::glColor3ub(Color_256::r(*color_itr), Color_256::g(*color_itr), Color_256::b(*color_itr));
  //    ++color_itr;
  //    ::glBegin(GL_LINES);
  //    // Iterates through point indices assigned to each detected shape.
  //    std::vector<std::size_t>::const_iterator
  //        index_it = (*it)->indices_of_assigned_points().begin();
  //    while (index_it != (*it)->indices_of_assigned_points().end()) {
  //        // Retrieves point
  //        const Point_with_normal &p = *(m_points.begin() + (*index_it));
  //        ::glVertex3d(p.first.x(), p.first.y(), p.first.z());
  //        ::glVertex3d(p.first.x() + p.second.x() * 0.05, p.first.y() + p.second.y() * 0.05, p.first.z() + p.second.z() * 0.05);
  //        // Proceeds with next point.
  //        index_it++;
  //    }
  //    // Proceeds with next detected shape.
  //    it++;
  //    ::glEnd();
  //}

  // bounding box to polyhedron
  /*Epic_Polyhedron bxply = BboxToPolyhedron<Kernel2>(m_bbox);
  CGAL::Polygon_mesh_slicer<Epic_Polyhedron, Kernel2> slicer(bxply);*/
  // draw slicer plane
  glLineWidth(3.0);
  glColor3f(0.8f, 0.8f, 0.8f);
  for (auto &s : shapes) {
    CGAL::Bbox_3 bx = m_points[*(s->indices_of_assigned_points().begin())].first.bbox();
    for (auto &p : s->indices_of_assigned_points())
      bx += m_points[p].first.bbox();
    Epic_Polyhedron bxply = BboxToPolyhedron<Kernel2>(bx);
    CGAL::Polygon_mesh_slicer<Epic_Polyhedron, Kernel2> slicer(bxply);

    Kernel2::Plane_3 p = shape_info_to_plane(*s);
    Polylines pls;
    slicer(p, std::back_inserter(pls));
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    //glBegin(GL_LINE_LOOP);
    for (auto &pl : pls)
      for (auto &p : pl)
        glVertex3d(p.x(), p.y(), p.z());
    glEnd();
  }

  ::glEnable(GL_LIGHTING);
}

Kernel2::Plane_3 shape_info_to_plane(const Efficient_ransac::Shape &s)
{
  std::stringstream sstm(s.info());
  std::string str;
  sstm >> str;
  sstm >> str;
  sstm >> str;
  double a = std::atof(str.substr(1, str.size() - 2).c_str());
  sstm >> str;
  double b = std::atof(str.substr(0, str.size() - 1).c_str());
  sstm >> str;
  double c = std::atof(str.substr(0, str.size() - 2).c_str());
  sstm >> str;
  sstm >> str;
  double d = std::atof(str.substr(0, str.size() - 1).c_str());

  //std::cout << a << " " << b << " " << c << " " << d << std::endl;
  return Kernel2::Plane_3(a, b, c, d);
}

}
