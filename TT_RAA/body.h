#ifndef BODY_H
#define BODY_H
#include <QString>
#include <vector>
#include "joint.h"

using namespace std;

class Joint;

typedef struct {
    int r;
    int g;
    int b;
} colorStruct;

typedef struct {
    float x;
    float y;
    float z;

} coordVector;

typedef struct {
    int c1 ;
    int c2 ;
    int c3 ;
} triFace;

typedef struct {
    int c1 ;
    int c2 ;
    int c3 ;
    int c4 ;
} quadFace;

typedef struct{
    coordVector *a;
    coordVector *b;
    coordVector *c;
    coordVector *n;
} Triangle;

class Body //: public PQP_Model
{
public:
    Body();
    ~Body();
    QString bodyName;

    // Dynamics for later usage
    // float mass, inertia[9];

    // Kinematics
    float center[3], transform[3], rotation[4];
    Joint *child;
    bool hasChild;
    bool isColliding;

    // Geometry and PQP
    float scale[3];
    //  PQP_Model *bodyPQP;
    vector <coordVector> points;
    vector <triFace> indexTriFaces;
    vector <quadFace> indexQuadFaces;
    vector <Triangle> faces;

    // openGL
    colorStruct color;
    vector <coordVector> triNormals;
    vector <coordVector> quadNormals;
    vector <int> indexNormals;

    // Set Methods
    void adjustPoints();
    void calculateNormals();
    coordVector makeUnitary(coordVector uNormal);
    void createFaces();


};

#endif // BODY_H
