#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
  void readSettings();
  void writeSettings();

  // quit & close
  void quit();
  void closeEvent(QCloseEvent *event);

  // file menu
  void on_actionLoadPolyhedron_triggered();

  // edit menu
  void on_actionSave_snapshot_triggered();
  void on_actionCopy_snapshot_triggered();

  // algorithm menu
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
