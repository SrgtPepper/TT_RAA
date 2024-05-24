#ifndef WORLDGL_H
#define WORLDGL_H
#include <QtOpenGL/QGLWidget>
#include "body.h"
#include "robot.h"
#include "gl.h"


using namespace std;

class Body;
class Robot;

using namespace std;

class WorldGL : public QGLWidget
{
    Q_OBJECT
GL jele ;
public:
    WorldGL(QWidget *parent = 0);
    ~WorldGL();
    vector <Body> bodyVector;
    vector <Robot> robotVector;
//    vector <Node> solVector;
    bool paintFlag, paintSolFlag, enable3d;


public slots:
    void setStrength3d(GLfloat value);
    void setXrotation(GLfloat angle);
    void setYrotation(GLfloat angle);
    void setXtranslation(GLfloat distance);
    void setYtranslation(GLfloat distance);
    void setZtranslation(GLfloat distance);
    bool isBodyVectorEmpty();
    bool isRobotVectorEmpty();
    void appendBody(Body iBody);
    void appendRobot(Robot iRobot);
    void removeBody();
    void removeRobot();
    void clearBodyVector();
    void clearRobotVector();
    bool checkCollisions();
    bool checkBodyCollisions(Body *pBody);
    void resetBodyCollisionFlag();
    void setRobotConfig(vector<float> config,int robotNumber);
//    void setSolutionPointers(vector<Node*> newSolVector);

protected:
    // Inherited QGLWidget methods
    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();
    // Methods

    void paintRobot(Robot pRobot);
    void paintBody(Body pBody, bool overrideColor = false);
    void recursivePaint(Joint *pJoint, float pCenter[3],
                        bool overrideColor = false,
                        bool initial = false,
                        bool final = false,
                        bool sol = false);
    bool recursiveCollision(Joint *pJoint, float pCenter[3]);


    // Interface methods
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

private:
    GLfloat transX, transY, transZ, rotX, rotY, strength3d;
    QPoint lastPos;

};

#endif // WORLDGL_H
