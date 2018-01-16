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
  const double xmin = bbox.xmin();
  const double ymin = bbox.ymin();
  const double zmin = bbox.zmin();
  const double xmax = bbox.xmax();
  const double ymax = bbox.ymax();
  const double zmax = bbox.zmax();
  qglviewer::Vec 
    vec_min(xmin, ymin, zmin),
    vec_max(xmax, ymax, zmax);
  viewer->setSceneBoundingBox(vec_min,vec_max);
  viewer->camera()->showEntireScene();
}

void Mainwindow::open(QString filename)
{
  QFileInfo fileinfo(filename);
  if(fileinfo.isFile() && fileinfo.isReadable())
  {
    int index = scene->open(filename);
    if(index >= 0)
    {
      QSettings settings;
      settings.setValue("OFF open directory",
        fileinfo.absoluteDir().absolutePath());
      this->addToRecentFiles(filename);

      // update bbox
      updateViewerBBox();
      viewer->update();
    }
  }
}

void Mainwindow::quit()
{
  writeSettings();
  close();
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

void Mainwindow::dropEvent(QDropEvent *event)
{
  Q_FOREACH(QUrl url, event->mimeData()->urls()) {
    QString filename = url.toLocalFile();
    if(!filename.isEmpty()) {
      QTextStream(stderr) << QString("dropEvent(\"%1\")\n").arg(filename);
      open(filename);
    }
  }
  event->acceptProposedAction();
}

void Mainwindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  event->accept();
}

void Mainwindow::dragEnterEvent(QDragEnterEvent *event)
{
  if (event->mimeData()->hasFormat("text/uri-list"))
    event->acceptProposedAction();
}

void Mainwindow::on_actionLoadPolyhedron_triggered()
{
  QSettings settings;
  QString directory = settings.value("OFF open directory",
    QDir::current().dirName()).toString();
  QStringList filenames = 
    QFileDialog::getOpenFileNames(this,
    tr("Load polyhedron..."),
    directory,
    tr("OFF files (*.off)\n"
    "All files (*)"));
  if(!filenames.isEmpty()) {
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
  QString directory = settings.value("OFF open directory",
    QDir::current().dirName()).toString();
  QStringList filenames =
    QFileDialog::getOpenFileNames(this,
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
  // Edit menu actions

  // Show menu actions
  /*QObject::connect(this->actionShow_Axis, SIGNAL(toggled(bool)),
    viewer, SLOT(toggleShowAxis(bool)));
  QObject::connect(this->actionShow_Vertex, SIGNAL(toggled(bool)),
    viewer, SLOT(toggleShowVertex(bool)));
  QObject::connect(this->actionShow_DEdge, SIGNAL(toggled(bool)),
    viewer, SLOT(toggleShowDEdge(bool)));
  QObject::connect(this->actionShow_VEdge, SIGNAL(toggled(bool)),
    viewer, SLOT(toggleShowVEdge(bool)));
  QObject::connect(this->actionShow_Facet, SIGNAL(toggled(bool)),
    viewer, SLOT(toggleShowFacet(bool)));
  QObject::connect(this->actionFlat, SIGNAL(toggled(bool)),
    viewer, SLOT(toggleFlat(bool)));*/

  // Preferences
  /*QObject::connect(this->actionPreferences, SIGNAL(triggered()),
    viewer, SLOT(setPreferences()));*/

  // connects actionQuit (Ctrl+Q) and qApp->quit()
  connect(actionQuit, SIGNAL(triggered()), this, SLOT(quit()));

  connect(this, SIGNAL(openRecentFile(QString)), this, SLOT(open(QString)));

  // Viewer signals
  /*QObject::connect(this, SIGNAL(sceneChanged()),
    viewer, SLOT(updateGL()));*/
}
