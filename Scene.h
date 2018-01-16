#ifndef SCENE_H
#define SCENE_H

#include <QtOpenGL/qgl.h>
#include <iostream>
#include <cmath>

#include "types.h"

class Scene
{
public:
  Scene();
  ~Scene();

public:
  // file menu
  int open(QString filename);

  void update_bbox();
  Bbox bbox() { return m_bbox; }

  // toggle view options
  void toggle_view_poyhedron() {
    m_view_polyhedron = !m_view_polyhedron;
  }

  // algorithms
  void refine_loop() {}
  void fit_triangles() {}
  void fit_edges() {}
  void fit_vertices() {}
  
  // RANSAC shape detection on point cloud algorithm
  int shape_detection(const std::string &fname);

  // triangulated surface mesh simplification algorithm
  //int surface_simplification();
  int surface_simplification(QString filename);

  // rendering
  void draw(); 
  void render_polyhedron();

private:
  // member data
  Polyhedron *m_pPolyhedron;
  Bbox m_bbox;

  // view options
  bool m_view_polyhedron;
}; // end class Scene


#endif // SCENE_H
