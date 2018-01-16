#include "dialSettings.h"

SettingsDialog::SettingsDialog(QWidget *parent)
  : QDialog(parent)
{
  setupUi(this);
  loadFromSettings();
}

void SettingsDialog::loadFromSettings()
{
  QSettings settings("settings.ini", QSettings::IniFormat);
  settings.beginGroup("APP");

  settings.endGroup();
}

void SettingsDialog::saveToSettings()
{
  QSettings settings("settings.ini", QSettings::IniFormat);
  settings.beginGroup("APP");

  settings.endGroup();
}

void SettingsDialog::accept()
{
  saveToSettings();

  QDialog::accept();
}
