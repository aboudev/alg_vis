#include "Settings_dialog.h"

Settings_dialog::Settings_dialog(QWidget *parent) :
  QDialog(parent)
{
  setupUi(this);
  loadFromSettings();
}

void Settings_dialog::loadFromSettings()
{
  QSettings settings("settings.ini", QSettings::IniFormat);
  settings.beginGroup("APP");

  settings.endGroup();
}

void Settings_dialog::saveToSettings()
{
  QSettings settings("settings.ini", QSettings::IniFormat);
  settings.beginGroup("APP");

  settings.endGroup();
}

void Settings_dialog::accept()
{
  saveToSettings();

  QDialog::accept();
}
