#include "body.h"
#include <math.h>

Body::Body()
{
    for (int i=0;i<3;i++){
        center[i]=0;
        transform[i]=0;
        rotation[i]=0;
    }
    rotation[3]=0;
    hasChild = false;
    isColliding = false;
}

Body::~Body()
{
}

//// Methods
void Body::adjustPoints()
{
    for(unsigned int i=0; i<points.size(); i++){
        points.at(i).x = points.at(i).x-center[0];
        points.at(i).y = points.at(i).y-center[1];
        points.at(i).z = points.at(i).z-center[2];
    }
}


void Body::calculateNormals()
{
    // Calculates normals for each triangle using cross product
    for(vector<triFace>::iterator triIt = indexTriFaces.begin(); triIt != indexTriFaces.end(); triIt++){
    triFace triFaceAux;
    triFaceAux = *triIt;

    coordVector u,v,newNormal;
    u.x=points.at(triFaceAux.c2).x-points.at(triFaceAux.c1).x;
    u.y=points.at(triFaceAux.c2).y-points.at(triFaceAux.c1).y;
    u.z=points.at(triFaceAux.c2).z-points.at(triFaceAux.c1).z;

    v.x=points.at(triFaceAux.c3).x-points.at(triFaceAux.c1).x;
    v.y=points.at(triFaceAux.c3).y-points.at(triFaceAux.c1).y;
    v.z=points.at(triFaceAux.c3).z-points.at(triFaceAux.c1).z;

    newNormal.x = (u.y * v.z) - (u.z * v.y);
    newNormal.y = (u.z * v.x) - (u.x * v.z);
    newNormal.z = (u.x * v.y) - (u.y * v.x);
    newNormal = makeUnitary(newNormal);
    triNormals.push_back(newNormal);
    }

    // Calculates normals for each quad face
    for(vector<quadFace>::iterator quadIt = indexQuadFaces.begin(); quadIt != indexQuadFaces.end(); quadIt++){
    quadFace quadFaceAux;
    quadFaceAux = *quadIt;

    coordVector u,v,newNormal;
    u.x=points.at(quadFaceAux.c2).x-points.at(quadFaceAux.c1).x;
    u.y=points.at(quadFaceAux.c2).y-points.at(quadFaceAux.c1).y;
    u.z=points.at(quadFaceAux.c2).z-points.at(quadFaceAux.c1).z;

    v.x=points.at(quadFaceAux.c4).x-points.at(quadFaceAux.c1).x;
    v.y=points.at(quadFaceAux.c4).y-points.at(quadFaceAux.c1).y;
    v.z=points.at(quadFaceAux.c4).z-points.at(quadFaceAux.c1).z;

    newNormal.x = (u.y * v.z) - (u.z * v.y);
    newNormal.y = (u.z * v.x) - (u.x * v.z);
    newNormal.z = (u.x * v.y) - (u.y * v.x);

    newNormal = makeUnitary(newNormal);
    quadNormals.push_back(newNormal);
    }
}

coordVector Body::makeUnitary(coordVector iNormal)
{
    coordVector uNormal;
    float mag, aux;
    aux = (iNormal.x*iNormal.x) + (iNormal.y*iNormal.y) + (iNormal.z*iNormal.z);
    mag = sqrt(aux);
    uNormal.x = iNormal.x / mag;
    uNormal.y = iNormal.y / mag;
    uNormal.z = iNormal.z / mag;
    return uNormal;
}

void Body::createFaces(){
        for (unsigned int i=0; i<indexTriFaces.size(); i++){
            triFace objTriFace = indexTriFaces[i];

            coordVector objNormal = triNormals[i];
            Triangle objTrian;

            objTrian.a = &points[objTriFace.c1];
            objTrian.b = &points[objTriFace.c2];
            objTrian.c = &points[objTriFace.c3];
            objTrian.n = &objNormal;

            faces.push_back(objTrian);
        }

   }
