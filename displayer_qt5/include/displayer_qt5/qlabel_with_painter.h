#ifndef displayer_qt5_QLABEL_WITH_PAINTER_H
#define displayer_qt5_QLABEL_WITH_PAINTER_H

#include <QMainWindow>
#include <QObject>
#include <QLabel>
#include <QPoint>
#include <QPainter>
#include "displayer_qt5/qnode.hpp"
#include <QPaintEvent>

class QLabel_with_painter : public QLabel
{
    Q_OBJECT
protected:
    void  paintEvent(QPaintEvent* );
private:
    QPoint placeRB1_te[4];
    QPoint placeRB2_te[7];
    QPoint placeRB3_te[5];
    QPoint placeLeap_te[4];
    QPoint placeHitWindMill_te[4];
    QPoint placeOutpose_te[4];
    QPoint placeRB1_en[4];
    QPoint placeRB2_en[7];
    QPoint placeRB3_en[5];
    QPoint placeLeap_en[4];
    QPoint placeHitWindMill_en[4];
    QPoint placeOutpose_en[4];
    std::vector<world_point> worldPoints;
public:
    explicit QLabel_with_painter(QWidget *parent = nullptr);
    void drawSmallMap(std::vector<world_point>& );
};

#endif // QLABEL_WITH_PAINTER_H
