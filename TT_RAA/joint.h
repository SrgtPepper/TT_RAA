#ifndef JOINT_H
#define JOINT_H
#include "body.h"
#include <vector>

using namespace std;

class Body;

typedef struct{
    float min;
    float max;
} limitStruct;

typedef struct{
    float x;
    float y;
    float z;
} jointAxisStruct;

class Joint
{
public:
    Joint();
    ~Joint();

    // Joint type :
    // 0 -> Rotational
    // 1 -> Prismatic
    // 10 -> Virtual joint for hierarchy purposes
    QString jointName;
    int type;
    float value, initial, final, solution;
    limitStruct limits;
    jointAxisStruct jointAxis;

    // Hierarchy
    bool hasFather, hasChildren;
    Body *father;
    vector <Body*> children;
    QString fatherString;
    vector <QString> childString;

};

#endif // JOINT_H

