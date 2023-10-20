#include "joint.h"

Joint::Joint()
{
        value = 0;
        initial = 0;
        final = 0;
        hasFather = false;
        hasChildren = false;
}

Joint::~Joint()
{
}
