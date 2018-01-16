#include "MainWindow.h"
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
#include <QMessageBox>
#include <qlabel.h>

#include "ui_MainWindow.h"

#include <QMimeData> 


MainWindow::MainWindow(QWidget* parent) :
  CGAL::Qt::DemosMainWindow(parent)
{
  ui = new Ui::MainWindow;
  ui->setupUi(this);

  // saves some pointers from ui, for latter use.
  m_pViewer = ui->viewer;

  // does not save the state of the viewer 
  m_pViewer->setStateFileName(QString::null);

  // accepts drop events
  setAcceptDrops(true);

  // setups scene
  m_pScene = new Scene;
  m_pViewer->setScene(m_pScene);

  ui->menuHelp->addAction(actionAbout);
  connectActions();

  this->addRecentFiles(ui->menuFile, ui->actionQuit);
  connect(this, SIGNAL(openRecentFile(QString)),
    this, SLOT(open(QString)));

  // About menu
  // addAboutCGAL() is a function in DemoMainWindow
  //   it will add a menu action "About CGAL..." to Help menu and connect to popupAboutCGAL
  //   default popupAboutCGAL points to a fixed file directory ":/cgal/help/about_CGAL.html"
  //   We can override it with our directory in function popupAboudCGAL() function
  this->addAboutCGAL();

  readSettings();
}

void MainWindow::popupAboutDemo()
{
  // overwrite popupAboutDemo to hard code the text
  QString about_txt("<h2>CGAL Algorithm Application</h2>");
  about_txt += "<p>Copyright &copy; 2016<br>";
  about_txt += "<a href = \"http://vision.ia.ac.cn\">IACAS Robot Vision Group<a/></p>";
  about_txt += "<p>This application is intended to illustrates algorithms implemented in ";
  about_txt += "<a href = \"http://www.cgal.org/\">CGAL</a>.</p>";
  about_txt += "<p>Author - Zlj</p>";
  QMessageBox mb(QMessageBox::NoIcon,
    tr("About Application..."),
    about_txt,
    QMessageBox::Ok,
    this);

  QLabel* mb_label = mb.findChild<QLabel*>("qt_msgbox_label");
  if (mb_label) {
    mb_label->setTextInteractionFlags(mb_label->textInteractionFlags() |
      ::Qt::LinksAccessibleByMouse |
      ::Qt::LinksAccessibleByKeyboard);
  }
  else {
    std::cerr << "Cannot find child \"qt_msgbox_label\" in QMessageBox\n"
      << "  with Qt version " << QT_VERSION_STR << "!\n";
  }
  mb.exec();

  return;
}

void MainWindow::connectActions()
{
  // Edit menu actions

  // Show menu actions
  /*QObject::connect(this->actionShow_Axis, SIGNAL(toggled(bool)),
    m_pViewer, SLOT(toggleShowAxis(bool)));
  QObject::connect(this->actionShow_Vertex, SIGNAL(toggled(bool)),
    m_pViewer, SLOT(toggleShowVertex(bool)));
  QObject::connect(this->actionShow_DEdge, SIGNAL(toggled(bool)),
    m_pViewer, SLOT(toggleShowDEdge(bool)));
  QObject::connect(this->actionShow_VEdge, SIGNAL(toggled(bool)),
    m_pViewer, SLOT(toggleShowVEdge(bool)));
  QObject::connect(this->actionShow_Facet, SIGNAL(toggled(bool)),
    m_pViewer, SLOT(toggleShowFacet(bool)));
  QObject::connect(this->actionFlat, SIGNAL(toggled(bool)),
    m_pViewer, SLOT(toggleFlat(bool)));*/

  // Preferences
  /*QObject::connect(this->actionPreferences, SIGNAL(triggered()),
    m_pViewer, SLOT(setPreferences()));*/

  // Help menu actions
  QObject::connect(ui->actionViewer_Help, SIGNAL(triggered()),
    m_pViewer, SLOT(help()));
  QObject::connect(actionAbout, SIGNAL(triggered()),
    this, SLOT(popupAboutDemo()));

  // connects actionQuit (Ctrl+Q) and qApp->quit()
  connect(ui->actionQuit, SIGNAL(triggered()),
    this, SLOT(quit()));

  // Viewer signals
  /*QObject::connect(this, SIGNAL(sceneChanged()),
    m_pViewer, SLOT(updateGL()));*/
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
  if (event->mimeData()->hasFormat("text/uri-list"))
    event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent *event)
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

void MainWindow::updateViewerBBox()
{
  m_pScene->update_bbox();
  const Scene::Bbox bbox = m_pScene->bbox();
  const double xmin = bbox.xmin();
  const double ymin = bbox.ymin();
  const double zmin = bbox.zmin();
  const double xmax = bbox.xmax();
  const double ymax = bbox.ymax();
  const double zmax = bbox.zmax();
  qglviewer::Vec 
    vec_min(xmin, ymin, zmin),
    vec_max(xmax, ymax, zmax);
  m_pViewer->setSceneBoundingBox(vec_min,vec_max);
  m_pViewer->camera()->showEntireScene();
}

void MainWindow::on_actionView_polyhedron_triggered()
{
  m_pScene->toggle_view_poyhedron();
  m_pViewer->update();
}

void MainWindow::on_actionRefine_loop_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  m_pScene->refine_loop();
  QApplication::restoreOverrideCursor();
  m_pViewer->update();
}


void MainWindow::open(QString filename)
{
  QFileInfo fileinfo(filename);
  if(fileinfo.isFile() && fileinfo.isReadable())
  {
    int index = m_pScene->open(filename);
    if(index >= 0)
    {
      QSettings settings;
      settings.setValue("OFF open directory",
        fileinfo.absoluteDir().absolutePath());
      this->addToRecentFiles(filename);

      // update bbox
      updateViewerBBox();
    m_pViewer->update();
    }
  }
}

void MainWindow::readSettings()
{
  this->readState("MainWindow", Size|State);
}

void MainWindow::writeSettings()
{
  this->writeState("MainWindow");
  std::cerr << "Write setting... done.\n";
}

void MainWindow::quit()
{
  writeSettings();
  close();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  event->accept();
}

void MainWindow::on_actionLoadPolyhedron_triggered()
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


void MainWindow::setAddKeyFrameKeyboardModifiers(::Qt::KeyboardModifiers m)
{
  m_pViewer->setAddKeyFrameKeyboardModifiers(m);
}

void MainWindow::on_actionSave_snapshot_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  m_pViewer->saveSnapshot(QString("snapshot.png"));
  QApplication::restoreOverrideCursor();
}
void MainWindow::on_actionCopy_snapshot_triggered()
{
  // copy snapshot to clipboard
  QApplication::setOverrideCursor(Qt::WaitCursor);
  QClipboard *qb = QApplication::clipboard();
  m_pViewer->makeCurrent();
  m_pViewer->raise();
  QImage snapshot = m_pViewer->grabFrameBuffer(true);
  qb->setImage(snapshot);
  QApplication::restoreOverrideCursor();
}

void MainWindow::on_actionFit_triangles_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  m_pScene->fit_triangles();
  m_pViewer->update();
  QApplication::restoreOverrideCursor();
}

void MainWindow::on_actionFit_edges_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  m_pScene->fit_edges();
  m_pViewer->update();
  QApplication::restoreOverrideCursor();
}

void MainWindow::on_actionFit_vertices_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  m_pScene->fit_vertices();
  m_pViewer->update();
  QApplication::restoreOverrideCursor();
}

void MainWindow::on_actionShape_detection_triggered()
{
  QApplication::setOverrideCursor(Qt::WaitCursor);
  m_pScene->shape_detection();
  m_pViewer->update();
  QApplication::restoreOverrideCursor();
}

void MainWindow::on_actionSurface_simplification_triggered()
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
  m_pScene->surface_simplification(filename);
  updateViewerBBox();
  m_pViewer->update();
  QApplication::restoreOverrideCursor();
}