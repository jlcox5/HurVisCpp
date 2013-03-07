/*
 * velocityFunct.h
 *
 *  Created on: Aug 12, 2010
 *  Jonathan Cox
 *  Clemson University
 */

#include "Vector.h"
#include "posPoint.h"

#ifndef VELOCITYFUNCT_H_
#define VELOCITYFUNCT_H_

Vector2d * genVelocity(double lon1, double lon2, double lat1, double lat2, double time);
Vector2d normVel(double x, double y);

#endif /* VELOCITYFUNCT_H_ */
