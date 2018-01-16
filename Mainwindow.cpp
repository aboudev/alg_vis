#include "Mainwindow.h"
#include "Scene.h"
#include <CGAL/Qt/debug.h>

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QTextStream>
#include <QUrl>
#include <QFileDialog>
#include <QSettings>
#include <QHeaderView>
#include <QClipboard>

#include <QMimeData>

Mainwindow::Mainwindow(QWidget *parent) :
  CGAL::Qt::DemosMainWindow(parent)
{
  setupUi(this);

  // does not save the state of the viewer 
  viewer->setStateFileName(QString::null);

  // setups scene
  scene = new Scene;
  viewer->setScene(scene);

  // accepts drop events
  setAcceptDrops(true);

  // add recent files actions before action quit
  addRecentFiles(menuFile, actionQuit);

  // connect actions
  connectActions();

  addAboutDemo(":/cgal/about/alg_vis.html");
  addAboutCGAL();

  readSettings();
}

Mainwindow::~Mainwindow()
{
  delete scene;
}

void Mainwindow::updateViewerBBox()
{
  scene->update_bbox();
  const Scene::Bbox bbox = scene->bbox();
  const qglviewer::Vec vec_min(
    bbox.xmin(), bbox.ymin(), bbox.zmin());
  const qglviewer::Vec vec_max(
    bbox.xmax(), bbox.ymax(), bbox.zmax());
  viewer->setSceneBoundingBox(vec_min,vec_max);
  viewer->camera()->showEntireScene();
}

void Mainwindow::open(QString filename)
{
  QFileInfo fileinfo(filename);
  if (fileinfo.isFile() && fileinfo.isReadable()) {
    if (scene->open(filename) >= 0) {
      QSettings settings;
      settings.setValue("OFF open directory", fileinfo.absoluteDir().absolutePath());
      addToRecentFiles(filename);

      // update bbox
      updateViewerBBox();
      viewer->update();
    }
  }
}

void Mainwindow::readSettings()
{
  this->readState("Mainwindow", Size|State);
}

void Mainwindow::writeSettings()
{
  this->writeState("Mainwindow");
  std::cerr << "Write setting... done.\n";
}

void Mainwindow::quit()
{
  writeSettings();
  close();
}

void Mainwindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  event->accept();
}

void Mainwindow::on_actionLoadPolyhedron_triggered()
{
  QSettings settings;
  QString directory = settings.value("OFF open directory", QDir::current().dirName()).toString();
  QStringList filenames = QFileDialog::getOpenFileNames(
    this,
    tr("Load polyhedron..."),
    directory,
    tr("OFF files (*.off)\n"
    "All files (*)"));
  if (!filenames.isEmpty()) {
    Q_FOREACH(QString filename, filenames) {
      open(filename);
    }
  }
}

void Mainwindow::on_actionSave_snapshot_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  viewer->saveSnapshot(QString("snapshot.png"));
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionCopy_snapshot_triggered()
{
  // copy snapshot to clipboard
  QApplication::setOverrideCursor(Qt::WaitCursor);
  QClipboard *qb = QApplication::clipboard();
  viewer->makeCurrent();
  viewer->raise();
  QImage snapshot = viewer->grabFrameBuffer(true);
  qb->setImage(snapshot);
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionRefine_loop_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  scene->refine_loop();
  QApplication::restoreOverrideCursor();
  viewer->update();
}
void Mainwindow::on_actionFit_triangles_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  scene->fit_triangles();
  viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionFit_edges_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  scene->fit_edges();
  viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionFit_vertices_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  scene->fit_vertices();
  viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionShape_detection_triggered()
{
  const QString fileName = QFileDialog::getOpenFileName(
    this,
    tr("Open point with normal"),
    ".",
    tr("Point Cloud File (*.pwn)"));

  QApplication::setOverrideCursor(Qt::WaitCursor);

  scene->shape_detection(fileName.toStdString());

  updateViewerBBox();
  viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionSurface_simplification_triggered()
{
  QSettings settings;
  QString directory = settings.value("OFF open directory", QDir::current().dirName()).toString();
  QStringList filenames = QFileDialog::getOpenFileNames(
    this,
    tr("Load surface mesh..."),
    directory,
    tr("OFF files (*.off)"));

  QString filename;
  if (!filenames.isEmpty())
    filename = *(filenames.begin());

  QApplication::setOverrideCursor(Qt::WaitCursor);
  scene->surface_simplification(filename);
  updateViewerBBox();
  viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionView_polyhedron_triggered()
{
  scene->toggle_view_poyhedron();
  viewer->update();
}

void Mainwindow::connectActions()
{
  // connects actionQuit (Ctrl+Q) and qApp->quit()
  connect(actionQuit, SIGNAL(triggered()), this, SLOT(quit()));

  connect(this, SIGNAL(openRecentFile(QString)), this, SLOT(open(QString)));
}
