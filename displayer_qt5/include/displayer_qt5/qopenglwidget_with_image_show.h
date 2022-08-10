#ifndef displayer_qt5_QOPENGLWIDGET_WITH_IMAGE_SHOW_H
#define displayer_qt5_QOPENGLWIDGET_WITH_IMAGE_SHOW_H

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLExtraFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLTexture>
#include <opencv2/opencv.hpp>
#include <QOpenGLShaderProgram>

class QOpenGlWidget_with_image_show : public QOpenGLWidget, public QOpenGLExtraFunctions
{
    Q_OBJECT
public:
    explicit QOpenGlWidget_with_image_show(QWidget *parent = nullptr);
    ~QOpenGlWidget_with_image_show();
    void setPicture(int, int);
    void updatePicture(QImage image);
protected:
    virtual void initializeGL() override;//初始化函数，在Widget刚加载时被调用
    virtual void resizeGL(int w, int h) override;//绘图函数，每一次绘图请求产生，都会进行一次绘图
    virtual void paintGL() override;//用于处理窗口大小变化的情况
private:
    QOpenGLBuffer m_vbo;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLShaderProgram *m_shader;//渲染器程序对象
    QOpenGLTexture *texture = nullptr;
};

#endif // QOPENGLWIDGET_WITH_IMAGE_SHOW_H
