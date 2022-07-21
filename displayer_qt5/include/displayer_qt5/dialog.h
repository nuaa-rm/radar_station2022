#ifndef displayer_qt5_DIALOG_H
#define displayer_qt5_DIALOG_H

#include <QMainWindow>
#include <QObject>
#include <QDialog>
#include <QTimer>

class Dialog : public QDialog
{
    Q_OBJECT
public:
    explicit Dialog(QWidget *parent = nullptr);
private:
    QTimer *fTimer;
    QTime fTimeCounter;
private slots:
    void on_timer_timeout();

};

#endif // DIALOG_H
