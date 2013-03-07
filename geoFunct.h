/*
 * geoFunct.h
 *
 *  Created on: Aug 12, 2010
 *  Jonathan Cox
 *  Clemson University
 */
#include <vector>
#include "Vector.h"
#include "posPoint.h"

#ifndef GEOFUNCT_H_
#define GEOFUNCT_H_

Vector2d translateToScreen(double curLon, double curLat);
Vector2d locateDestination(double curLon, double curLat, double velX, double velY, double curDeg);
Vector2d locateDestination_2(double curLon, double curLat, double dist, double curDeg);
double haversine(double lon1, double lon2, double lat1, double lat2);
double findBearing_2(Vector2d * pos1, Vector2d * pos2);
double crossTrack(Vector4d * projPath1, Vector4d * projPath2, Vector4d curPoint);

std::vector<Vector4d*> smoothCurve(Vector4d * position1, Vector4d * position2, double deg1, double deg2, double ts, int numSeg);

double findDif(double a, double b);
#endif /* GEOFUNCT_H_ */
