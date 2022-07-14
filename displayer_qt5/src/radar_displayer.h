#ifndef RADAR_DISPLAYER_H
#define RADAR_DISPLAYER_H

#include <QMainWindow>

namespace Ui {
class radar_displayer;
}

class radar_displayer : public QMainWindow
{
    Q_OBJECT

public:
    explicit radar_displayer(QWidget *parent = nullptr);
    ~radar_displayer();

private:
    Ui::radar_displayer *ui;
};

#endif // RADAR_DISPLAYER_H
