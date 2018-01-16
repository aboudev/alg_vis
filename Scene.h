#ifndef SCENE_H
#define SCENE_H

#include <QtOpenGL/qgl.h>
#include <iostream>
#include <cmath>

#include "types.h"

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

class Scene
{
public:
  Scene();
  ~Scene();
public:
  // types
  typedef CGAL::Bbox_3 Bbox;

public:
  void update_bbox();
  Bbox bbox() { return m_bbox; }

private:
  // member data
  Bbox m_bbox;
  Line m_line;
  Plane m_plane;
  Point m_centroid;
  Polyhedron *m_pPolyhedron;

  // view options
  bool m_view_polyhedron;

public:
  // file menu
  int open(QString filename);

  // toggle view options
  void toggle_view_poyhedron();

  // algorithms
  Vector normalize(const Vector& v);

  void refine_loop();
  void fit_edges();
  void fit_vertices();
  void fit_triangles();

  // RANSAC shape detection on point cloud algorithm
  int shape_detection(const std::string &fname);

  // triangulated surface mesh simplification algorithm
  //int surface_simplification();
  int surface_simplification(QString filename);

  // rendering
  void draw(); 
  void render_line();
  void render_plane();
  void render_centroid();
  void render_polyhedron();

  void render_shape();

private:
  // Points with normals.
  Pwn_vector m_points;

  Efficient_ransac m_ransac;

  // dummy class for shapes_range initialization
  // defined before m_bestshapes
  // http://stackoverflow.com/questions/4037219/order-of-execution-in-constructor-initialization-list
  Dummy m_dm;

  Efficient_ransac::Shape_range m_bestshapes;

  // view shapes.
  bool m_view_shapes;
}; // end class Scene


#endif // SCENE_H
