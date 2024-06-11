#include <QtOpenGL/QtOpenGL>
#include <QtWidgets>
#include "worldgl.h"
#include "gl.h"
#include <iostream>


WorldGL::WorldGL(QWidget *parent)
    : QGLWidget(parent)
{
    // Con esto se modifica el viewpoint
    transX = 0.0f;
    transY = -35.0f;
    transZ = -120.0f;
    rotX = -60.0f;
    rotY = 0.0f;
    strength3d = 2.5f;
    paintFlag = false;
    paintSolFlag = false;
    enable3d = false;
}

WorldGL::~WorldGL()
{
    clearBodyVector();
    clearRobotVector();
}

void WorldGL::setStrength3d(GLfloat value)
{
    strength3d = value;
}

void WorldGL::setXrotation(GLfloat angle)
{
    rotX=angle;
}

void WorldGL::setYrotation(GLfloat angle)
{
    rotY=angle;
}

void WorldGL::setXtranslation(GLfloat distance)
{
    transX = distance;
}

void WorldGL::setYtranslation(GLfloat distance)
{
    transY = distance;
}

void WorldGL::setZtranslation(GLfloat distance)
{
    transZ=distance;
}

bool WorldGL::isBodyVectorEmpty()
{
    if (bodyVector.empty())
    {
        return true;
    }
    else {
        return false;
    }
}

bool WorldGL::isRobotVectorEmpty()
{
    if (robotVector.empty())
    {
        return true;
    }
    else {
        return false;
    }
}

void WorldGL::appendBody(Body iBody)
{
    bodyVector.push_back(iBody);
}

void WorldGL::appendRobot(Robot iRobot)
{
    robotVector.push_back(iRobot);
    Robot *robotAux;
    robotAux = &(robotVector.back());
    robotAux->getHierarchy();

}

void WorldGL::removeBody()
{
    bodyVector.pop_back();
}

void WorldGL::removeRobot()
{
    robotVector.pop_back();
}

void WorldGL::clearBodyVector()
{
    bodyVector.clear();
}

void WorldGL::clearRobotVector()
{
    robotVector.clear();
}

void WorldGL::setRobotConfig(vector<float> config, int robotNumber){
    Robot *pRobot;
    pRobot = &(robotVector.at(robotNumber));
    pRobot->setConfig(config);
}

void WorldGL::initializeGL()
{
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

void WorldGL::resizeGL(int w, int h)
{
    int side = qMin(w,h);
    glViewport((w-side)/2,(h-side)/2,side,side);

    // Use Perspective Camera Model
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Perspective Camera Configuration (FOV,Aspect Ratio, near, far)
    jele.perspectiveGL(45.0f,(GLfloat)w/(GLfloat)h,0.1f,10000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void WorldGL::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Camera Transform
    glTranslatef(transX, transY, transZ);

    // Render twice to make anaglyph 3d
    if (enable3d){
        glPushMatrix();

        // LEFT EYE
        glTranslatef(-strength3d, 0.0, 0.0);
        glRotatef(rotX, 1.0, 0.0, 0.0);
        glRotatef(rotY, 0.0, 1.0, 0.0);
        glColorMask(GL_FALSE, GL_TRUE, GL_TRUE, GL_TRUE);

        if (!bodyVector.empty()) {
            for (vector<Body>::iterator it = bodyVector.begin(); it != bodyVector.end() ; it++){
                paintBody(*it);
            }
        }
        // Paint every robot
        if (!robotVector.empty()) {
            Robot robotAux;
            for (unsigned int i=0; i<robotVector.size(); i++){
                robotAux = (robotVector.at(i));
                paintRobot(robotAux);
            }
        }
        glPopMatrix();

        glClear(GL_DEPTH_BUFFER_BIT);
        // RIGHT EYE
        glPushMatrix();
        glTranslatef(strength3d,0.0,0.0);
        glRotatef(rotX, 1.0, 0.0, 0.0);
        glRotatef(rotY, 0.0, 1.0, 0.0);
        glColorMask(GL_TRUE, GL_FALSE, GL_FALSE, GL_TRUE);

        if (!bodyVector.empty()) {
            for (vector<Body>::iterator it = bodyVector.begin(); it != bodyVector.end() ; it++){
                paintBody(*it);
            }
        }
        // Paint every robot
        if (!robotVector.empty()) {
            Robot robotAux;
            for (unsigned int i=0; i<robotVector.size(); i++){
                robotAux = (robotVector.at(i));
                paintRobot(robotAux);
            }
        }
        glPopMatrix();
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    }
    else{
        glRotatef(rotX, 1.0, 0.0, 0.0);
        glRotatef(rotY, 0.0, 1.0, 0.0);
        // Paint every body of environment
        if (!bodyVector.empty()) {
            for (vector<Body>::iterator it = bodyVector.begin(); it != bodyVector.end() ; it++){
                paintBody(*it);
            }
        }
        // Paint every robot
        if (!robotVector.empty()) {
            Robot robotAux;
            for (unsigned int i=0; i<robotVector.size(); i++){
                robotAux = (robotVector.at(i));
                paintRobot(robotAux);
            }
        }
        // Paint solution

        if (paintSolFlag) {
            Robot *robotAux;
            robotAux = &(robotVector.at(0));
            glColor3ub(100,100,100);
            glPushMatrix();
            glTranslatef(robotAux->transform.x,
                         robotAux->transform.y,
                         robotAux->transform.z);

            glPopMatrix();
        }
    }
}

void WorldGL::paintRobot(Robot robotAux)
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

void WorldGL::recursivePaint(Joint *pJoint, float pCenter[3], bool overrideColor, bool initial, bool final, bool sol)
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

void WorldGL::paintBody(Body pBody, bool overrideColor)
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

/*
void WorldGL::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
    updateGL();
}

void WorldGL::mouseMoveEvent(QMouseEvent *event)
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

void WorldGL::wheelEvent(QWheelEvent *event)
{
    int dz = event->delta();
    setZtranslation(transZ + dz);
    updateGL();
}
*/

bool WorldGL::checkCollisions()
{
    bool collisionExists = false;
    resetBodyCollisionFlag();
    glLoadIdentity();
    if(!robotVector.empty()){
        Robot *robotAux;
        Joint *firstJoint;
        float pCenter[3] = {0,0,0};
        for (unsigned int i=0; i<robotVector.size();i++){
            robotAux = &(robotVector.at(i));
            robotAux->resetBodyCollisionFlag();
            // Translate to world position
            glPushMatrix();
            glTranslatef(robotAux->transform.x,
                         robotAux->transform.y,
                         robotAux->transform.z);
            for (unsigned int j=0; j<robotAux->firstJoint.size();j++){
                firstJoint = robotAux->firstJoint.at(j);
                if (recursiveCollision(firstJoint, pCenter)){
                    collisionExists = true;
                }
            }
            glPopMatrix();
        }
    }
    return collisionExists;
}

bool WorldGL::recursiveCollision(Joint *pJoint, float pCenter[3])
{
    bool collisionExists = false;
    glPushMatrix();

    // Select Joint Children
    Body *pBody;
    for (unsigned int i=0; i<pJoint->children.size(); i++){
        pBody = pJoint->children.at(i);

        // Translate to Center
        glTranslatef(pBody->center[0]-pCenter[0],
                     pBody->center[1]-pCenter[1],
                     pBody->center[2]-pCenter[2]);

        // Transform according to joint type
        float value;
        value = pJoint->value;
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

        // Check the selected body collisions against environment
        if (checkBodyCollisions(pBody)){
            collisionExists = true;
        }

        // Recursive call to explore the hierarchy
        if (pBody->hasChild){
            Joint *childJoint;
            childJoint = pBody->child;
            float newCenter[3] = {pBody->center[0],
                                  pBody->center[1],
                                  pBody->center[2]};
            if (recursiveCollision(childJoint,newCenter)){
                collisionExists = true;
            }
        }
        glPopMatrix();
    }
    return collisionExists;
}

bool WorldGL::checkBodyCollisions(Body *pBody)
{
    bool collisionExists = false;
    Body *bodyEnv;
    // Body static Transforms
    glPushMatrix();
    glTranslatef(pBody->transform[0],
                 pBody->transform[1],
                 pBody->transform[2]);
    glRotatef(pBody->rotation[0],
              pBody->rotation[1],
              pBody->rotation[2],
              pBody->rotation[3]);


    GLdouble modelView[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelView);

    for (unsigned int i=0; i<bodyVector.size(); i++){
        bodyEnv = &(bodyVector.at(i));

    }
    glPopMatrix();
    return collisionExists;
}

void WorldGL::resetBodyCollisionFlag()
{
    for (unsigned int i=0; i<bodyVector.size(); i++)
    {
        Body *bodyAux;
        bodyAux = &(bodyVector.at(i));
        bodyAux->isColliding = false;
    }
}

