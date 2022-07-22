/**
 * @file /include/displayer_qt5/main_window.hpp
 *
 * @brief Qt based gui for displayer_qt5.
 *
 * @date November 2010
 **/
#ifndef displayer_qt5_QLABEL_WITH_MOUSE_EVENT_H
#define displayer_qt5_QLABEL_WITH_MOUSE_EVENT_H

#include <QMainWindow>
#include <QObject>
#include <QLabel>
#include <QPoint>
#include <QMouseEvent>
#include <QPainter>
#include "qnode.hpp"

class QLabel_with_mouse_event : public QLabel
{
    Q_OBJECT
protected:
    void  mouseMoveEvent(QMouseEvent *event);
    void  mousePressEvent(QMouseEvent *event);
    void  mouseReleaseEvent(QMouseEvent *event);
    void  paintEvent(QPaintEvent* );
private:

    bool if_point_being_selected;
public:
    explicit QLabel_with_mouse_event(QWidget *parent = nullptr);
    QPoint selectedPoint;
    QString cameraCelibrating;
    QString sensorFarImgRaw;
    QString sensorCloseImgRaw;
    QPoint* sensor_far_points;
    QPoint* sensor_close_points;
    bool if_is_dragging;
signals:
    void mouseMovePoint(QPoint point);
    void mouseClicked(QPoint point);
    void mouseReleased(QPoint point);
};

#endif // displayer_qt5_QLABEL_WITH_MOUSE_EVENT_H
