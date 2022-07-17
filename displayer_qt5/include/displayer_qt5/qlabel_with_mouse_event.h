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

class QLabel_with_mouse_event : public QLabel
{
    Q_OBJECT
protected:
    void  mouseMoveEvent(QMouseEvent *event);
    void  mousePressEvent(QMouseEvent *event);
public:
    explicit QLabel_with_mouse_event(QWidget *parent = nullptr);
signals:
    void mouseMovePoint(QPoint point);
    void mouseClicked(QPoint point);
};

#endif // displayer_qt5_QLABEL_WITH_MOUSE_EVENT_H
