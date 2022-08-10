#include "displayer_qt5/qopenglwidget_with_image_show.h"
#include <QPainter>
#include <QOpenGLWidget>
#include <iostream>

QOpenGlWidget_with_image_show::QOpenGlWidget_with_image_show(QWidget *parent) : QOpenGLWidget(parent)
{

}

QOpenGlWidget_with_image_show::~QOpenGlWidget_with_image_show()
{
//    //多线程调用保护
//    makeCurrent();
//    //对象释放
//    m_vbo->destroy();
//    m_vao->destroy();
//    delete texture;
//    //退出保护
//    doneCurrent();
}

//void QOpenGlWidget_with_image_show::paintEvent(QPaintEvent *event)
//{
//    QPainter painter;
//    painter.begin(this);
//    painter.drawImage(QPoint(0, 0), imageToShow);
//    painter.end();

//    imageData_ = nullptr;
//}


void QOpenGlWidget_with_image_show::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);   // 三维绘图的关键！
    glClearColor(0, 0.5, 0.7, 1);
    m_shader = new QOpenGLShaderProgram();
    m_shader->addShaderFromSourceCode(QOpenGLShader::Vertex, u8R"(
       #version 140
       in vec3 vPos;
       in vec2 vTexture;
       out vec2 oTexture;
       void main()
       {
           gl_Position = vec4(vPos, 1.0);
           oTexture = vTexture;
       }

    )");
    m_shader->addShaderFromSourceCode(QOpenGLShader::Fragment, u8R"(
       #version 140
       in vec2 oTexture;
       uniform sampler2D uTexture;
       void main()
       {
           gl_FragColor = texture(uTexture, oTexture);
       }

    )");
    m_shader->link();
//    float _vertex[] =
//    {
//        0.0, 0.5, 0.0,
//        0.5, 0.0, 0.0,
//        -0.5, 0.0, 0.0,
//    };
//    static const GLfloat vertices[] = {
//            //位置                //纹理位置
//        -1.0f,  1.0f, 0.0f,  0.0f, 1.0f  // top left
//        -1.0f, -1.0f, 0.0f,  0.0f, 0.0f, // bottom left
//        1.0f, -1.0f, 0.0f,  1.0f, 0.0f, // bottom right
//        1.0f,  1.0f, 0.0f,  1.0f, 1.0f, // top right
//    };
    float _vertex[] = {
        //  顶点			纹理
            -1,  1, 0,	0, 1, // 左上
            -1, -1, 0,	0, 0, // 左下
             1, -1, 0,	1, 0, // 右下
             1,  1, 0,	1, 1, // 右上
        };
    m_vao.create();
    m_vbo.create();
    m_vao.bind();
    m_vbo.bind();
    m_vbo.allocate(_vertex, sizeof (_vertex));
    m_shader->bind();
    m_shader->setAttributeBuffer("vPos", GL_FLOAT, 0 * sizeof (float), 3, 5 * sizeof (float));
    m_shader->enableAttributeArray("vPos");
    m_shader->setAttributeBuffer("vTexture", GL_FLOAT, 3 * sizeof (float), 2, 5 * sizeof (float));
    m_shader->enableAttributeArray("vTexture");
        //VBO数据

    texture = new QOpenGLTexture(QImage("/home/dovejh/image.png").mirrored());

    m_shader->release();
    m_vao.release();
}

void QOpenGlWidget_with_image_show::resizeGL(int width, int height)
{

}

void QOpenGlWidget_with_image_show::paintGL()
{
    texture->bind();
    m_vao.bind();
    m_shader->bind();
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    m_shader->release();
    m_vao.release();
    texture->release();
}

void QOpenGlWidget_with_image_show::updatePicture(QImage image)
{
    texture->destroy();
    texture->setData(image.mirrored());
    update();
}

void QOpenGlWidget_with_image_show::setPicture(int w, int h)
{
//    std::cout << "1" << std::endl;
//    texture = new QOpenGLTexture(QOpenGLTexture::Target2D);
//    texture->create();
//    texture->setSize(w, h);//设置纹理宽、高
//    texture->setLayers(1); //设置个数
//    texture->setFormat(QOpenGLTexture::RGB8_UNorm);//设置纹理像素格式
    //texture->setMinificationFilter(QOpenGLTexture::Linear);//设置放大缩小采样方式
    //texture->setMagnificationFilter(QOpenGLTexture::Linear);
    //texture->allocateStorage();
}

