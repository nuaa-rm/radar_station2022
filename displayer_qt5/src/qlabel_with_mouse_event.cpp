#include "displayer_qt5/qlabel_with_mouse_event.h"
#include <iostream>
void QLabel_with_mouse_event::mouseMoveEvent(QMouseEvent *event)
{
    QPoint point = event->pos();
    emit mouseMovePoint(point);
    QLabel::mouseMoveEvent(event);
    selectedPoint = point;
    //std::cout << point.x() << '\t' << point.y() << std::endl;
}

void QLabel_with_mouse_event::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPoint point = event -> pos();
        emit mouseClicked(point);
        update();
    }
    QLabel::mousePressEvent(event);
}

void QLabel_with_mouse_event::mouseReleaseEvent(QMouseEvent *event)
{

}

void QLabel_with_mouse_event::paintEvent(QPaintEvent *event)
{
    QLabel::paintEvent(event);
    QPainter painter(this);
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(3);
    painter.setPen(pen);
    painter.drawEllipse(selectedPoint, 10, 10);
}


QLabel_with_mouse_event::QLabel_with_mouse_event(QWidget *parent) : QLabel{parent}
{
    setMouseTracking(true);
    selectedPoint = QPoint(0, 0);
}

