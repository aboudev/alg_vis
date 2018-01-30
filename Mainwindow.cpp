#include "Mainwindow.h"
#include "Scene.h"
#include "Settings_dialog.h"

#include <QFileDialog>
#include <QSettings>
#include <QClipboard>

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
  if (scene)
    delete scene;
}

void Mainwindow::updateViewerBBox()
{
  const Bbox_3 bbox = scene->bbox();
  const qglviewer::Vec vec_min(
    bbox.xmin(), bbox.ymin(), bbox.zmin());
  const qglviewer::Vec vec_max(
    bbox.xmax(), bbox.ymax(), bbox.zmax());
  viewer->setSceneBoundingBox(vec_min,vec_max);
  viewer->camera()->showEntireScene();
}

void Mainwindow::open(QString filename)
{
  QApplication::setOverrideCursor(QCursor(::Qt::WaitCursor));

  QFileInfo fileinfo(filename);
  if (fileinfo.isFile() && fileinfo.isReadable()) {
    if (scene->open(filename.toStdString()) >= 0) {
      QSettings settings;
      settings.setValue("OFF open directory", fileinfo.absoluteDir().absolutePath());
      addToRecentFiles(filename);

      // update bbox
      updateViewerBBox();
      viewer->update();
    }
  }

  QApplication::restoreOverrideCursor();
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
  const QString filename = QFileDialog::getOpenFileName(
    this,
    tr("Load polyhedron..."),
    settings.value("off_open_directory", ".").toString(),
    tr("OFF files (*.off)\nAll files (*)"));
  if (!filename.isEmpty())
    return;
  settings.setValue("off_open_directory", filename);

  open(filename);
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

void Mainwindow::on_actionSurface_simplification_triggered()
{
  QSettings settings;
  const QString filename = QFileDialog::getOpenFileName(
    this,
    tr("Load surface mesh..."),
    settings.value("surface_simplification_open_directory", ".").toString(),
    tr("OFF files (*.off)"));
  if (filename.isEmpty())
    return;
  settings.setValue("surface_simplification_open_directory", filename);

  QApplication::setOverrideCursor(Qt::WaitCursor);

  scene->surface_simplification(filename.toStdString());

  // updateViewerBBox();
  // viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionShape_detection_triggered()
{
  QSettings settings;
  const QString filename = QFileDialog::getOpenFileName(
    this,
    tr("Open point with normal"),
    settings.value("shape_detection_open_directory", ".").toString(),
    tr("Point Cloud With Normal (*.ply *.pwn)"));
  if (filename.isEmpty())
    return;
  settings.setValue("shape_detection_open_directory", filename);

  Settings_dialog dial;
  dial.shape_detection->setEnabled(true);
  if (dial.exec() != QDialog::Accepted)
    return;

  QApplication::setOverrideCursor(Qt::WaitCursor);
  Params::Shape_detection params{
    dial.shape_detection_probability->value(),
    static_cast<std::size_t>(dial.shape_detection_min_points->value()),
    dial.shape_detection_epsilon->value(),
    dial.shape_detection_cluster_epsilon->value(),
    dial.shape_detection_normal_threshold->value()};

  scene->shape_detection(filename.toStdString(), params);

  updateViewerBBox();
  viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionRidge_detection_triggered()
{
  QSettings settings;
  const QString filename = QFileDialog::getOpenFileName(
    this,
    tr("Triangle mesh"),
    settings.value("ridge_detection_open_directory", ".").toString(),
    tr("OFF files (*.off)"));
  if (filename.isEmpty())
    return;
  settings.setValue("ridge_detection_open_directory", filename);

  // Settings_dialog dial;
  // dial.shape_detection->setEnabled(true);
  // if (dial.exec() != QDialog::Accepted)
  //   return;

  QApplication::setOverrideCursor(Qt::WaitCursor);
  // Params::Shape_detection params{
  //   dial.shape_detection_probability->value(),
  //   static_cast<std::size_t>(dial.shape_detection_min_points->value()),
  //   dial.shape_detection_epsilon->value(),
  //   dial.shape_detection_cluster_epsilon->value(),
  //   dial.shape_detection_normal_threshold->value()};

  scene->ridge_detection(filename.toStdString());

  updateViewerBBox();
  viewer->update();
  QApplication::restoreOverrideCursor();
}

void Mainwindow::on_actionCustom_plane_detection_triggered()
{
  QSettings settings;
  const QString filename = QFileDialog::getOpenFileName(
    this,
    tr("Open point with normal"),
    settings.value("custom_plane_detection_open_directory", ".").toString(),
    tr("Point Cloud With Normal (*.ply *.pwn)"));
  if (filename.isEmpty())
    return;
  settings.setValue("custom_plane_detection_open_directory", filename);

  Settings_dialog dial;
  dial.cplane_detection->setEnabled(true);
  if (dial.exec() != QDialog::Accepted)
    return;

  QApplication::setOverrideCursor(Qt::WaitCursor);
  Params::Shape_detection params{
    dial.cplane_detection_probability->value(),
    static_cast<std::size_t>(dial.cplane_detection_min_points->value()),
    dial.cplane_detection_epsilon->value(),
    dial.cplane_detection_cluster_epsilon->value(),
    dial.cplane_detection_normal_threshold->value()};

  scene->custom_plane_detection(filename.toStdString(), params);

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
