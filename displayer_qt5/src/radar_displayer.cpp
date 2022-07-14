#include "radar_displayer.h"
#include "ui_radar_displayer.h"

radar_displayer::radar_displayer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::radar_displayer)
{
    ui->setupUi(this);
}

radar_displayer::~radar_displayer()
{
    delete ui;
}
