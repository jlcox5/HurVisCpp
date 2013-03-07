/* Jonathan Cox
   Clemson University

   Implements the chip class
*/

#include <iostream>
#include <vector>      
#include <GL/glu.h>

#include "chip.h"

chip::chip(double v, double x, double y, double r){
   value = v;
   pos.set(x, y);
   initPos.set(x, y);
   rad = r*7;
   isMoving = 0;
   sector = -1;
   draw = true;
}

void chip::selectChip(Vector2d p){
   double xd, yd, dist;

   if(draw == true){
     xd = abs(p.x - pos.x);
     yd = abs(p.y - pos.y);
     dist = sqrt(xd*xd + yd*yd);

     if(dist < rad){
       isMoving = 1;
     }
     else{
       isMoving = 0;
     }
   }
}

int chip::releaseChip(Vector2d p, std::vector<Vector2d*> se, std::vector<double> tl, double r){
   std::vector<double>::iterator _d1;
   std::vector<double>::iterator _d2;
   Vector2d pos2, u1, u2;
   double theta, x, y, angle_radians;
   int foundSector = 0;
   int s = 0;
   int initSect = sector;

   // Test distance to make sure it's in the sector
   if( isMoving != 0 && (pos-p).norm() < r){
      // Check to see if the chip is in any sectors
      u1 = (pos-p).normalize();
   
      angle_radians = 0.0*(double)M_PI/180.0;
      x = p.x + rad*(double)sin(angle_radians);
      y = p.y + -1*rad*(double)cos(angle_radians);
      pos2.set(x, y);
      u2 = (pos2-p).normalize();
      theta = (acos(u1*u2)*180.0)/M_PI;

      if(pos.x < p.x){
         theta = 360.0 - theta;
      }
      else if(pos.x == p.x){
         if(pos.y == p.y){
            theta = 0.0;
         }
         else{
            theta = 180.0;
         }
      }

      _d2 = tl.begin()+1;
      for(_d1 = tl.begin(); _d2 != tl.end(); _d1++){
         // Normal Sector
         //cout << "d1: " << (*_d1) << "   d2: " << (*_d2) << "  t: " << theta << "  s: " << s << endl;
         if((*_d1) < (*_d2)){
            if(theta > (*_d1) && theta <= (*_d2)){
               sector = s;
            }
         }
         // Sector crosses 0 degrees
         else{
            if(theta > (*_d1) || theta < (*_d2)){
               sector = s;
            }
         }
         s++;
         _d2++;
      }
      // Test last sector
      _d1 = tl.end()-1;
      _d2 = tl.begin();
      if((*_d1) < (*_d2)){
         if(theta > (*_d1) && theta <= (*_d2)){
            sector = s;
         }
      }
      // Sector crosses 0 degrees
      else{
         if(theta > (*_d1) || theta < (*_d2)){
            sector = s;
         }
      }

      //cout << "In sector: " << sector << endl;
      if(sector == initSect && sector != -1){
         pos = sectorPos;
      }
   }
   else if(isMoving != 0 && (pos-p).norm() > r){
      sector = -1;
      //cout << "In sector: " << sector << endl;
   }

   isMoving = 0;
   return sector;
}

void chip::moveChip(Vector2d v){
   if(isMoving == 1){
     pos = pos + v;
   }
}

void chip::printPos(){
   cout << pos << endl;
}


void chip::drawChip(){
   int angle;
   double angle_radians;
   double x, y;
   double x1, y1;

  if(draw == true){
    glColor4f(0.75, 0.0, 0.75, 1.0);
    glLineWidth(1.5);
    glBegin(GL_LINE_LOOP);
    for(angle = 0; angle < 360; angle += 5){
      angle_radians = angle*(double)M_PI/180.0;
      x = pos.x + rad*(double)cos(angle_radians);
      y = pos.y + rad*(double)sin(angle_radians);
      glVertex3f(x,y,0.0); 
    }
    glEnd();

    glColor4f(0.75, 0.0, 0.75, 0.5);
    x1 = y1 = 0.0;
    glBegin(GL_POLYGON);
    for(angle = 0; angle < 360; angle += 5){
      angle_radians = angle*(double)M_PI/180.0;
      x = pos.x + rad*(double)cos(angle_radians);
      y = pos.y + rad*(double)sin(angle_radians);
      if(angle == 0){
        x1 = x; 
        y1 = y;
      }
      glVertex3f(x,y,0.0); 
    }
    glVertex3f(x1,y1,0.0); 
    glEnd();
  }
}

void chip::setDrawChip(bool const b){
  draw = b;
}
