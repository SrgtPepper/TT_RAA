#include <QtOpenGL/QtOpenGL>
#include <QtOpenGL/QGLWidget>
#include <QtWidgets>
#include "gl.h"

GL::GL(QWidget *parent)
    : QGLWidget(parent)
{
    transX = 0.0f;
    transY = 0.0f;
    transZ = -100.0f;
    rotX = 45.0f;
    rotY = 180.0f;
    paintFlag = false;
    enable3d = false;
    paintPath = false;
//    path.clear();
}

GL::~GL()
{
    robotVector.clear();
    bodyVector.clear();
//    path.clear();
}

void GL::setXrotation(GLfloat angle)
{
    rotX=angle;
}

void GL::setYrotation(GLfloat angle)
{
    rotY=angle;
}

void GL::setXtranslation(GLfloat distance)
{
    transX = distance;
}

void GL::setYtranslation(GLfloat distance)
{
    transY = distance;
}

void GL::setZtranslation(GLfloat distance)
{
    transZ=distance;
}

void GL::initializeGL()
{
//    glClearColor(255.0f , 255.0f, 255.0f, 255.0f);  // White Background
    glClearColor(0.0, 0.0, 0.0, 255.0); // Black Background
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // Lighting configuration
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glEnable(GL_LIGHT0);
    static GLfloat lightPos[4] = { 1000.0, 1000.0, 1000.0, 0.0 };
    static GLfloat ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    static GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };
    static GLfloat specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
    glLightfv(GL_LIGHT0,GL_POSITION,lightPos);

    GLfloat global_ambient[] = { 0.25f, 0.25f, 0.25f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);

}

void GL::resizeGL(int w, int h)
{
    int side = qMin(w,h);
    glViewport((w-side)/2,(h-side)/2,side,side);

    // Use Perspective Camera Model
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Perspective Camera Configuration (FOV,Aspect Ratio, near, far)


    //gluPerspective(45.0f,(GLfloat)w/(GLfloat)h,0.1f,10000.0f);
    perspectiveGL(45.0f,(GLfloat)w/(GLfloat)h,0.1f,10000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void GL::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Camera Transform
    glTranslatef(transX, transY, transZ);
    glRotatef(rotX, 1.0, 0.0, 0.0);
    glRotatef(rotY, 0.0, 1.0, 0.0);
    glPushMatrix();
//    drawPlane(100);
    if (!robotVector.empty()) {
               Robot robotAux;
               for (unsigned int i=0; i<robotVector.size(); i++){
                   robotAux = (robotVector.at(i));
                   paintRobot(robotAux);
               }
           }
    glPopMatrix();
//    if (paintPath){
//        drawPath();
//    }

}

void GL::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
    updateGL();
}

void GL::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();


    if (event->buttons() & Qt::LeftButton) {
        setXrotation(rotX + dy);
        setYrotation(rotY + dx);
    }

    if (event->buttons() & Qt::RightButton) {
        setXtranslation(transX + dx);
        setYtranslation(transY - dy);
    }

    lastPos = event->pos();
    updateGL();
}

void GL::wheelEvent(QWheelEvent *event)
{
    int dz = event->delta();
    setZtranslation(transZ + dz);
    updateGL();
}

//void GL::drawPlane(float size)
//{
//    glColor3ub(100,100,100);
//    glNormal3f(0.0,1.0,0.0);
//    glBegin(GL_QUADS);
//    glVertex3f(size, -1.0f, size);
//    glVertex3f(size, -1.0f, -size);
//    glVertex3f(-size, -1.0f, -size);
//    glVertex3f(-size, -1.0f, size);
//    glEnd();
//}


void GL::appendRobot(Robot iRobot)
{
    robotVector.clear();
    robotVector.push_back(iRobot);
    Robot *robotAux;
    robotAux = &(robotVector.back());
    robotAux->getHierarchy();

}

void GL::setRobotConfig(vector<float> config, int robotNumber){
    Robot *pRobot;
    pRobot = &(robotVector.at(robotNumber));
    pRobot->setConfig(config);
}

void GL::paintRobot(Robot robotAux)
{
    // Paints the robot in the initial position
    if (paintFlag){
        glColor4ub(50, 150, 50, 200);
        glPushMatrix();
        glTranslatef(robotAux.transform.x,
                     robotAux.transform.y,
                     robotAux.transform.z);
        for (unsigned int i=0; i<robotAux.firstJoint.size();i++){
            Joint *firstJoint;
            firstJoint = robotAux.firstJoint.at(i);
            float pCenter[3] = {0, 0, 0};
            recursivePaint(firstJoint, pCenter, true, true, false);
        }
        glPopMatrix();
        // Paints the robot in the final position with diferent color
        glPushMatrix();
        glColor4ub(50,50,150, 200);
        glTranslatef(robotAux.transform.x,
                     robotAux.transform.y,
                     robotAux.transform.z);
        for (unsigned int i=0; i<robotAux.firstJoint.size();i++){
            Joint *firstJoint;
            firstJoint = robotAux.firstJoint.at(i);
            float pCenter[3] = {0, 0, 0};
            recursivePaint(firstJoint, pCenter, true, false, true);
        }
        glPopMatrix();
    }

    if (!paintFlag){
                glColor3ub(200,200,200);
        glPushMatrix();
        glTranslatef(robotAux.transform.x,
                     robotAux.transform.y,
                     robotAux.transform.z);
        for (unsigned int i=0; i<robotAux.firstJoint.size();i++){
            Joint *firstJoint;
            firstJoint = robotAux.firstJoint.at(i);
            float pCenter[3] = {0, 0, 0};
            recursivePaint(firstJoint, pCenter, true);
        }
        glPopMatrix();
    }
}

void GL::recursivePaint(Joint *pJoint, float pCenter[3], bool overrideColor, bool initial, bool final, bool sol)
{
    glPushMatrix();

    // Paint Joint Children
    for(unsigned int i=0; i<pJoint->children.size(); i++){
        Body pBody;
        pBody = *(pJoint->children.at(i));

        // TEST FOR CENTER MOVEMENT
        glTranslatef(pBody.center[0]-pCenter[0],
                     pBody.center[1]-pCenter[1],
                     pBody.center[2]-pCenter[2]);
        float value;
        value = pJoint->value;
        if (initial) { value = pJoint->initial; }
        if (final) { value = pJoint->final; }
        if (sol) { value = pJoint->solution; }
        if (pJoint->type==0){
            glRotatef(value,
                      pJoint->jointAxis.x,
                      pJoint->jointAxis.y,
                      pJoint->jointAxis.z);
        }
        if (pJoint->type==1){
            glTranslatef(value * pJoint->jointAxis.x,
                         value * pJoint->jointAxis.y,
                         value * pJoint->jointAxis.z);
        }
        paintBody(pBody, overrideColor);
        if (pBody.hasChild){
            Joint *childJoint;
            childJoint = pBody.child;
            float newpCenter[3] = {pBody.center[0],
                                   pBody.center[1],
                                   pBody.center[2]};
            recursivePaint(childJoint, newpCenter, true, initial, final, sol);
        }
    }
    glPopMatrix();
}

void GL::paintBody(Body pBody, bool overrideColor)
{
    int i;
    // Body transform
    glPushMatrix();
    glTranslatef(pBody.transform[0], pBody.transform[1], pBody.transform[2]);
    glRotatef(pBody.rotation[0], pBody.rotation[1], pBody.rotation[2], pBody.rotation[3]);

    // Get Object Color if parameter overrideColor is false
    if (!overrideColor){
        glColor3ub(pBody.color.r,pBody.color.g,pBody.color.b);
    }
    if (pBody.isColliding){
        glColor3ub(255,0,0);
    }


    // Triangles
    i=0;
    glBegin(GL_TRIANGLES);
    for (vector<triFace>::iterator triIt = pBody.indexTriFaces.begin(); triIt != pBody.indexTriFaces.end() ; triIt++){
        triFace tFaceAux;
        tFaceAux = *triIt;
        glNormal3f((GLfloat)pBody.triNormals.at(i).x,
                   (GLfloat)pBody.triNormals.at(i).y,
                   (GLfloat)pBody.triNormals.at(i).z);

        glVertex3f((GLfloat)pBody.points.at(tFaceAux.c1).x,
                   (GLfloat)pBody.points.at(tFaceAux.c1).y,
                   (GLfloat)pBody.points.at(tFaceAux.c1).z);

        glVertex3f((GLfloat)pBody.points.at(tFaceAux.c2).x,
                   (GLfloat)pBody.points.at(tFaceAux.c2).y,
                   (GLfloat)pBody.points.at(tFaceAux.c2).z);

        glVertex3f((GLfloat)pBody.points.at(tFaceAux.c3).x,
                   (GLfloat)pBody.points.at(tFaceAux.c3).y,
                   (GLfloat)pBody.points.at(tFaceAux.c3).z);
        i++;
    }
    glEnd();
    glPopMatrix();
}

void GL::perspectiveGL( GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar )
{
    const GLdouble pi = 3.1415926535897932384626433832795;
    GLdouble fW, fH;

    //fH = tan( (fovY / 2) / 180 * pi ) * zNear;
    fH = tan( fovY / 360 * pi ) * zNear;
    fW = fH * aspect;

    glFrustum( -fW, fW, -fH, fH, zNear, zFar );
}

//void GL::drawPath()
//{
//    glColor3ub(255,0,0);
//    glBegin(GL_LINE_STRIP);
//    for (unsigned int i=0; i<path.size(); i++)
//        glVertex3f(path.at(i).y, 0 , path.at(i).x);
//    glEnd();
//}

