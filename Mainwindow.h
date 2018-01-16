#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtOpenGL/qgl.h>
#include <CGAL/Qt/DemosMainWindow.h>
#include "ui_Mainwindow.h"

class QDragEnterEvent;
class QDropEvent;
class Scene;

class Mainwindow :
  public CGAL::Qt::DemosMainWindow,
  public Ui_Mainwindow
{
  Q_OBJECT

public:
  Mainwindow(QWidget *parent = nullptr);
  ~Mainwindow();

public slots:
  void updateViewerBBox();
  void open(QString filename);

protected slots:
  // settings
  void quit();
  void readSettings();
  void writeSettings();

  // drag & drop
  void dropEvent(QDropEvent *event);
  void closeEvent(QCloseEvent *event);
  void dragEnterEvent(QDragEnterEvent *event);

  // file menu
  void on_actionLoadPolyhedron_triggered();

  // edit menu
  void on_actionSave_snapshot_triggered();
  void on_actionCopy_snapshot_triggered();

  // algorithm menu
  void on_actionRefine_loop_triggered();
  void on_actionFit_triangles_triggered();
  void on_actionFit_edges_triggered();
  void on_actionFit_vertices_triggered();
  void on_actionShape_detection_triggered();
  void on_actionSurface_simplification_triggered();

  // view menu
  void on_actionView_polyhedron_triggered();

private:
  void connectActions();

private:
  Scene *scene;
};

#endif // ifndef MAINWINDOW_H
