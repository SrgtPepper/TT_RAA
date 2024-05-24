#ifndef GL_H
#define GL_H
#include "body.h"
#include "robot.h"
#include <QtOpenGL/QGLWidget>

class Body;
class Robot;

using namespace std;

class GL : public QGLWidget
{
    Q_OBJECT

public:
    GL(QWidget *parent = 0);
    ~GL();

    bool paintFlag, enable3d, paintPath;
    vector <Body> bodyVector;
    vector <Robot> robotVector;
//    vector <q> path;

    void setXrotation(GLfloat angle);
    void setYrotation(GLfloat angle);
    void setXtranslation(GLfloat distance);
    void setYtranslation(GLfloat distance);
    void setZtranslation(GLfloat distance);

    void appendRobot(Robot iRobot);
    void setRobotConfig(vector<float> config,int robotNumber);
    void perspectiveGL( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar );


protected:
    // Inherited QGLWidget methods
    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    void paintRobot(Robot pRobot);
    void paintBody(Body pBody, bool overrideColor = false);
    void recursivePaint(Joint *pJoint, float pCenter[3],
                        bool overrideColor = false,
                        bool initial = false,
                        bool final = false,
                        bool sol = false);


    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    GLfloat transX, transY, transZ, rotX, rotY, strength3d;
    QPoint lastPos;
//    void drawPlane(float size);
//    void drawPath();
};

#endif // GL_H

