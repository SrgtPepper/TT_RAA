#ifndef ROBOT_H
#define ROBOT_H
#include <list>
#include <vector>
#include "body.h"
#include "joint.h"
#include "stdlib.h"
using namespace std;

class Body;
class Joint;

class Robot
{
public:
    Robot();
    ~Robot();

    // Methods
    void appendBody(Body iBody);
    void appendJoint(Joint iJoint);
    void getHierarchy();
//    void randomConfig();
    void setConfig(vector <float> jointValues);
    void setFullConfig(vector <float> jointValues);
    void getJointLimits();
    void resetBodyCollisionFlag();
    void adjustIniFin();
    void setSol(vector<float> jointValues);
    vector <float> getConfig();
    vector <float> getInitials();
    vector <float> getFinals();

    vector <int> initialPos;
    vector <int> finalPos;
    vector <Body> bodyVector;
    vector <Joint> jointVector;
    vector <Joint*> firstJoint;
    coordVector transform;
    vector <int> jointLimitsMin, jointLimitsMax;


};

#endif // ROBOT_H

