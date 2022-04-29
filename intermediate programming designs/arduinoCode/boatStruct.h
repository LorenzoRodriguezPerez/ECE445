#ifndef BOATSTRUCT_H
#define BOATSTRUCT_H

struct boatData{
  float yaw, pitch, roll, heading;
  float lat, lon;
  float base_lat, base_lon;

};

// b2 = copy of b1
void deepCopyBoat(boatData *b1, boatData *b2);

#endif