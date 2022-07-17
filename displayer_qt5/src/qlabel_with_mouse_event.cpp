#include "displayer_qt5/qlabel_with_mouse_event.h"

void QLabel_with_mouse_event::mouseMoveEvent(QMouseEvent *event)
{
    QPoint point = event->pos();
    emit mouseMovePoint(point);
    QLabel::mouseMoveEvent(event);
}

void QLabel_with_mouse_event::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPoint point = event -> pos();
        emit mouseClicked(point);
    }
    QLabel::mousePressEvent(event);
}

QLabel_with_mouse_event::QLabel_with_mouse_event(QWidget *parent) : QLabel{parent}
{
    setMouseTracking(true);
}

