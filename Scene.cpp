#include "Scene.h"

#include <iostream>
#include <fstream>

#include <QApplication>
#include <QString>
#include <QTextStream>
#include <QFileInfo>
#include <QInputDialog>

#include <CGAL/IO/Polyhedron_iostream.h>

int Scene::open(QString filename)
{
  QTextStream cerr(stderr);
  cerr << QString("Opening file \"%1\"\n").arg(filename);
  QApplication::setOverrideCursor(QCursor(::Qt::WaitCursor));

  QFileInfo fileinfo(filename);
  std::ifstream in(filename.toUtf8());

  if (!in || !fileinfo.isFile() || ! fileinfo.isReadable()) {
    std::cerr << "unable to open file" << std::endl;
    QApplication::restoreOverrideCursor();
    return -1;
  }

  if (m_pPolyhedron != nullptr)
    delete m_pPolyhedron;

  // allocate new polyhedron
  m_pPolyhedron = new Polyhedron;
  in >> *m_pPolyhedron;
  if (!in) {
    std::cerr << "invalid OFF file" << std::endl;
    QApplication::restoreOverrideCursor();

    delete m_pPolyhedron;
    m_pPolyhedron = nullptr;

    return -1;
  }

  QApplication::restoreOverrideCursor();
  return 0;
}

void Scene::update_bbox()
{
  std::cerr << "update bbox" << std::endl;
  if (m_pPolyhedron == nullptr || m_pPolyhedron->empty()) {
    std::cout << "(no / empty polyhedron)." << std::endl;
    return;
  }

  std::cout << "Compute bbox...";
  Polyhedron::Point_iterator it = m_pPolyhedron->points_begin();
  m_bbox = (*it).bbox();
  for (; it != m_pPolyhedron->points_end();it++)
    m_bbox = m_bbox + (*it).bbox();
  std::cout << "done (" << m_pPolyhedron->size_of_facets()
    << " facets)" << std::endl;
}

int Scene::shape_detection(const std::string &fname)
{
  return 0;
}

int Scene::surface_simplification(QString filename)
{
  return EXIT_SUCCESS;
}

void Scene::draw()
{
  if (m_view_polyhedron)
    render_polyhedron();
}

void Scene::render_polyhedron()
{
  if (!m_pPolyhedron)
    return;

  // draw black edges
  ::glDisable(GL_LIGHTING);
  ::glColor3ub(0, 0, 0);
  ::glLineWidth(1.0f);
  ::glBegin(GL_LINES);
  for (auto he = m_pPolyhedron->edges_begin(); he != m_pPolyhedron->edges_end(); ++he) {
    const Point& a = he->vertex()->point();
    const Point& b = he->opposite()->vertex()->point();
    ::glVertex3d(a.x(),a.y(),a.z());
    ::glVertex3d(b.x(),b.y(),b.z());
  }
  ::glEnd();
}
