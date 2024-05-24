#include "robot.h"

Robot::Robot()
{
    transform.x = 0;
    transform.y = 0;
    transform.z = 0;

}

Robot::~Robot()
{
    bodyVector.clear();
    jointVector.clear();
}

void Robot::appendBody(Body iBody)
{
    bodyVector.push_back(iBody);
}

void Robot::appendJoint(Joint iJoint)
{
    jointVector.push_back(iJoint);
}

void Robot::setConfig(vector <float> jointValues){
    Joint *jointAux;
    for (unsigned int i=1; i<jointVector.size(); i++){
        jointAux = &(jointVector.at(i));
        jointAux->value = jointValues.at(i-1);
    }
}

void Robot::setFullConfig(vector <float> jointValues){
    Joint *jointAux;
    for (unsigned int i=0; i<jointVector.size();i++){
        jointAux = &(jointVector.at(i));
        jointAux->value = jointValues.at(i);
    }
}

void Robot::getHierarchy()
{
    Joint *jointAux;
    Body *bodyAux;
    QString aux, aux2;
    // First assign pointers to father and child of each joint
    for (unsigned int i=0; i<jointVector.size();i++){
        jointAux = &jointVector.at(i);

        // Get father pointers for each joint
        aux = jointAux->fatherString;
        if (aux.contains("Null")){
            jointAux->hasFather = false;
            firstJoint.push_back(jointAux);
        }
        else {
            for (unsigned int j=0; j<bodyVector.size();j++){
                bodyAux = &bodyVector.at(j);
                aux2 = bodyAux->bodyName;
                if (aux.compare(aux2)==0){
                    jointAux->father = bodyAux;
                    jointAux->hasFather = true;
                    bodyAux->child = jointAux;
                    bodyAux->hasChild = true;
                }
            }
        }

        // Get children pointers for each joint
        for (unsigned int j=0; j<jointAux->childString.size();j++){
            aux = jointAux->childString.at(j);
            for (unsigned int k=0; k<bodyVector.size(); k++){
                bodyAux = &bodyVector.at(k);
                aux2 = bodyAux->bodyName;
                if (aux.compare(aux2)==0){
                    jointAux->children.push_back(bodyAux);
                    jointAux->hasChildren = true;
                }
            }
        }
    }

    // Now build a vector of pointers to joints according to hierarchy

}

void Robot::resetBodyCollisionFlag()
{
    for(unsigned int j=0; j<bodyVector.size();j++){
        Body* pointerBody;
        pointerBody = &(bodyVector.at(j));
        pointerBody->isColliding = false;
    }
}

void Robot::getJointLimits()
{
    for (unsigned int i=1; i<jointVector.size(); i++){
        Joint *pJoint;
        pJoint = &(jointVector.at(i));
        int max = pJoint->limits.max;
        int min = pJoint->limits.min;
        jointLimitsMax.push_back(max);
        jointLimitsMin.push_back(min);
    }
}

void Robot::adjustIniFin()
{
    Joint *pJoint;
    for (unsigned int i=0;i<initialPos.size();i++){
        pJoint = &(jointVector.at(i+1));
        pJoint->initial = initialPos.at(i);
        pJoint->final = finalPos.at(i);
    }
}

vector <float> Robot::getConfig()
{
    vector <float> jointValues;
    float value;
    Joint *pJoint;
    for (unsigned int i=0; i<jointVector.size(); i++){
        pJoint = &(jointVector.at(i));
        value = pJoint->value;
        jointValues.push_back(value);
    }
    return jointValues;
}

vector <float> Robot::getInitials()
{
    vector <float> jointValues;
    float value;
    Joint *pJoint;
    for (unsigned int i=0; i<jointVector.size();i++){
        pJoint = &(jointVector.at(i));
        value = pJoint->initial;
        jointValues.push_back(value);
    }
    return jointValues;
}

vector <float> Robot::getFinals()
{
    vector <float> jointValues;
    float value;
    Joint *pJoint;
    for (unsigned int i=0; i<jointVector.size();i++){
        pJoint = &(jointVector.at(i));
        value = pJoint->final;
        jointValues.push_back(value);
    }
    return jointValues;
}

void Robot::setSol(vector <float> jointValues){
    Joint *jointAux;
    for (unsigned int i=0; i<jointVector.size();i++){
        jointAux = &(jointVector.at(i));
        jointAux->solution = jointValues.at(i);
    }
}

