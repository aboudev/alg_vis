#ifndef SCENE_H
#define SCENE_H

#include <QtOpenGL/qgl.h>
#include <iostream>
#include <cmath>

#include "types.h"

class Scene
{
public:
  Scene() : m_pPolyhedron(nullptr), m_view_polyhedron(false) {}

  ~Scene() {
    if (m_pPolyhedron)
      delete m_pPolyhedron;
  }

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
