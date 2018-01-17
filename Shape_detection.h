////////////////////////////////////////////////////
// Author: ZLJ
// Date: 2018-01-16
// RVG, NLPR, CASIA
////////////////////////////////////////////////////

#ifndef Scene_shape_detection_H
#define Scene_shape_detection_H

#include "types.h"
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>

namespace Algs {

/************************************************************************/
/* RANSAC Shape Detection Algorithm                                     */
/* http://doc.cgal.org/latest/Point_set_shape_detection_3/index.html    */
/************************************************************************/
class Shape_detection {
  // Type declarations
  typedef std::pair<Kernel2::Point_3, Kernel2::Vector_3>       Point_with_normal;
  typedef std::vector<Point_with_normal>                       Pwn_vector;
  typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
  typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
  // In Shape_detection_traits the basic types, i.e., Point and Vector types
  // as well as iterator type and property maps, are defined.
  typedef CGAL::Shape_detection_3::Shape_detection_traits
    <Kernel2, Pwn_vector, Point_map, Normal_map>               Traits;
  typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>    Efficient_ransac;
  typedef CGAL::Shape_detection_3::Plane<Traits>               RansacPlane;

  class Dummy {
  public:
    Dummy(Efficient_ransac &r) {
      std::cerr << "in dummy" << std::endl;
      r.set_input(Pwn_vector(10, Point_with_normal()));
      /*r.add_shape_factory<RansacPlane>();
      r.preprocess();
      r.detect();*/
      std::cerr << "dummy done" << std::endl;
    }
  };

public:
  Shape_detection();

  const Bbox & bbox() { return m_bbox; }

  void detect(const std::string &fname, const Params::Shape_detection &params);

  void draw();

private:
  Kernel2::Plane_3 shape_info_to_plane(const Efficient_ransac::Shape &s);

  template <typename K>
  CGAL::Polyhedron_3<K> BboxToPolyhedron(const CGAL::Bbox_3 &bbox) {
    typedef typename CGAL::Polyhedron_3<K>::Point_3 Point;
    typedef typename CGAL::Polyhedron_3<K>::Halfedge_handle Halfedge_handle;

    Point p00(bbox.xmin(), bbox.ymin(), bbox.zmin());
    Point p01(bbox.xmax(), bbox.ymin(), bbox.zmin());
    Point p02(bbox.xmax(), bbox.ymax(), bbox.zmin());
    Point p03(bbox.xmin(), bbox.ymax(), bbox.zmin());
    Point p10(bbox.xmin(), bbox.ymin(), bbox.zmax());
    Point p11(bbox.xmax(), bbox.ymin(), bbox.zmax());
    Point p12(bbox.xmax(), bbox.ymax(), bbox.zmax());
    Point p13(bbox.xmin(), bbox.ymax(), bbox.zmax());

    CGAL::Polyhedron_3<K> p;
    Halfedge_handle h = p.make_tetrahedron(p01, p10, p00, p03);
    Halfedge_handle g = h->next()->opposite()->next();
    p.split_edge(h->next());
    p.split_edge(g->next());
    p.split_edge(g);
    h->next()->vertex()->point() = p11;
    g->next()->vertex()->point() = p13;
    g->opposite()->vertex()->point() = p02;
    Halfedge_handle f = p.split_facet(g->next(), g->next()->next()->next());
    Halfedge_handle e = p.split_edge(f);
    e->vertex()->point() = p12;
    p.split_facet(e, f->next()->next());
    CGAL_postcondition(p.is_valid());

    return p;
  }


private:
  Bbox m_bbox;

  // Points with normals.
  Pwn_vector m_points;

  Efficient_ransac m_ransac;

  // dummy class for shapes_range initialization
  // defined before m_bestshapes
  // http://stackoverflow.com/questions/4037219/order-of-execution-in-constructor-initialization-list
  Dummy m_dm;

  Efficient_ransac::Shape_range m_bestshapes;
};

} // namespace Algs

#endif // Scene_shape_detection_H
