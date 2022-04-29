#ifndef PID_H
#define PID_H

#include <boatStruct.h>
#include <control.h>


class PID{
public:
    void control(float desiredHeading, boatData * boat, Control& controlSail);

}

#endif