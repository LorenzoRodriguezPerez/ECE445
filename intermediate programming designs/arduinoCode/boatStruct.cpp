#include <boatStruct.h>

void deepCopyBoat(boatData *b1, boatData *b2){
    b2 -> yaw = b1 -> yaw;
    b2 -> pitch = b1 -> pitch;
    b2 -> roll = b1 -> roll;
    b2 -> heading = b1 -> heading;
    b2 -> lat = b1 -> lat;
    b2 -> lon = b1 -> lon;
    b2 -> base_lat = b1 -> base_lon;
    b2 -> base_lon = b1 -> base_lon;
}