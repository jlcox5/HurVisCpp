/*
 *  hurEyePoints.h
 *
 *  Created on: Feb 12, 2013
 *  Jonathan Cox
 *  Clemson University
 *
 *  This class simply generates random points for the eye of the hurricane that fall within
 *  the radius of the error cone with respect to the appropriate point on the predicted path.
 */

#include <climits>

#include "hurEyePoints.h"
#include "advisory.h"


extern double errorRadR[];

// For random number generation
extern unsigned int counter;

hurEyePoints::hurEyePoints(predictedPath * p){
  std::vector<Vector4d*>::iterator _p;
  trans = 1.0;
  int i = 0;
  int j = 0;
  int r = 0;
  double chosen = 0.0;
  double tempDis = 0.0;
  double tempBear = 0.0;
  int count = 0;

  Vector2d tempPos;
  Vector2d * toAdd;

  for(_p = p->predPathThreeHour.begin(); _p != p->predPathThreeHour.end(); _p++){
     count = 0;

     // Control how many number of points to generate for each spt on the error cone
     if(i == 3){ count = 1; }
     if(i == 7){ count = 1; }
     if(i == 11){ count = 2; }
     if(i == 15){ count = 3; }
     if(i == 23){ count = 4; }

     if(i == 3){ count = 1; }
     if(i == 7){ count = 1; }
     if(i == 11){ count = 1; }
     if(i == 15){ count = 1; }
     if(i == 23){ count = 1; }

     // Keep track of index into error cone radius array
     if(count > 0){ j++; }


     // Generate points
     for(int k = 0; k < count; k++){
       srand ( counter );
       counter = (counter*21)%UINT_MAX;

       r = rand() % 10000000+1;
       chosen = (double)r/10000000.0;
       tempDis = chosen*errorRadR[j];

       r = rand() % 10000000+1;
       chosen = (double)r/10000000.0;
       tempBear = chosen*360.0;

       /*if(i == 23){
         cout << "   hurEyePoints I: " << i << flush;
         cout <<  "   errorRadR: " << errorRadR[j] << flush;
         cout << "    tempDis: " << tempDis << "  tempBear: " << tempBear << endl << flush;

         cout << "    " << (*_p)->x;
         cout << "    " << (*_p)->y;
         cout << endl;
       }*/

       tempPos = locateDestination_2((*_p)->x, (*_p)->y, tempDis, tempBear);
       tempPos = translateToScreen(tempPos.x, tempPos.y);
       toAdd = new Vector2d;
       toAdd->x = tempPos.x;
       toAdd->y = tempPos.y;
       eyePositions.push_back(toAdd);
     }
     i++;
  }
}

void hurEyePoints::drawPoints(){
  std::vector<Vector2d*>::iterator _e;
  double angle_radians;
  int angle;

  float redVal = 0.5;
  float greenVal = 0.0;
  float blueVal = 0.0;

  int i = 0;
  for(_e = eyePositions.begin(); _e != eyePositions.end(); _e++){
    double x, y;
    double x1, y1;

    float newDropRadius = 3 + 7*(1.0 - trans);
    glColor4f(redVal, greenVal, blueVal, trans);
    x1 = y1 = 0.0;
 
    //glLineWidth(1.0 + 2*(1.0 - trans));
    //glBegin(GL_LINE_LOOP);
    glBegin(GL_POLYGON);
      for(angle = 0; angle < 360; angle += 5){
        angle_radians = angle*(double)M_PI/180.0;
        x = (*_e)->x + newDropRadius*(double)cos(angle_radians);
        y = (*_e)->y + newDropRadius*(double)sin(angle_radians);
        if(angle == 0){
          x1 = x; 
          y1 = y;
        }
        glVertex3f(x,y,0.0); 
      }
      glVertex3f(x1,y1,0.0); 
    glEnd();
    //redVal += 0.1;
    greenVal += 0.1;
    blueVal += 0.1;
    i++;

    if(_e == eyePositions.end()-1 && trans < 0){
      cout << "trans: " << trans << endl << flush;
    }
  } 
}
