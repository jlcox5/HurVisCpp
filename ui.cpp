/*
 *  ui.cpp
 *
 *  Created on: May 7, 2013
 *  Jonathan Cox
 *  Clemson University
 *
 *  This is the container class to hold all of the ui elements.  Ideally, each element should
 *  be derived from a uiElement base class, but we'll see if I can make that work shortly.  It
 *  might be a good... programming exercise. =)
 */

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#  include <GL/glu.h>
#endif

#include <iostream>
#include "ui.h"
#include "simulation.h"
#include "advisory.h"

extern simulation * sim;

ui::ui(){
   // Build chip stack
   chip * c;
   int px, py;
   int i, chipNum, chipVal;
   double radVal, pRadVal;
   //std::vector<int> * sectList;
   px = py = 45;
   chipNum = 14;
   chipVal = 0;
   radVal = 0;
   pRadVal = 4.5 * 7;

   for(i = 0; i < chipNum; i++){

      if(i < 2){ chipVal = 20; radVal = 4.5; }
      else if(i < 6){ chipVal = 10; pRadVal = radVal; radVal = 3.16;}
      else if(i < 9){ chipVal = 5; pRadVal = radVal; radVal = 2.23; }
      else{ chipVal = 1; pRadVal = radVal; radVal = 1; }

      // Set positions - x
      if(i == 0 || i == 2 || i == 6 || i == 9){
         px = 823 + radVal*7;
      }
      else{
         px += (radVal * 7) * 2 + 4;
      }

      // Set Positions - y
      if(i == 2 || i == 6 || i == 9){
         py += (pRadVal*7) + (radVal * 7) + 4;
      }

      c = new chip(chipVal, px, py, radVal);
      chips.push_back(c);
   }

   // Set sector positions
   /*for(i = 0; i < 8; i++){
      sectList = new std::vector<int>();
      for(j = 0; j < chipNum; j++){
         sectList->push_back(0); 
      }
      //sectorFull.push_back(sectList);
   }*/

   rad = 250;
   buildTargetArea();
   buildSectors();
}

void ui::buildTargetArea(){
   int angle;
   double angle_radians;
   double x, y;

   Vector2d pos;
   Vector2d * push;

   pos = translateToScreen(sim->adv->projPath[0]->x, sim->adv->projPath[0]->y); 

   for(angle = 0; angle < 360; angle += 5){
      angle_radians = angle*(double)M_PI/180.0;
      x = pos.x + rad*(double)cos(angle_radians);
      y = pos.y + rad*(double)sin(angle_radians);
      push = new Vector2d(x, y);
      targetArea.push_back(push);
   }
}

void ui::buildSectors(){
   int i;
   double angle_radians;
   double x, y;
   double theta;

   Vector2d pos0, pos1, pos2, u1, u2;
   Vector2d pos;
   Vector2d * push;

   sector * s;

   pos0 = translateToScreen(sim->adv->projPath[0]->x, sim->adv->projPath[0]->y); 
   pos1 = translateToScreen(sim->adv->projPath[2]->x, sim->adv->projPath[2]->y); 
   u1 = (pos1-pos0).normalize();
   u1.set(0.0, -1.0);

   x = pos0.x + rad*u1.x;
   y = pos0.y + rad*u1.y;

   push = new Vector2d(x, y);
   sectorEnds.push_back(push);

   theta = 332.5;

   thetaList.push_back(theta);

   for(i = 0; i < 7; i++){
      theta = fmod(theta + 45.0, 360.0);
      thetaList.push_back(theta);
      angle_radians = theta*(double)M_PI/180.0;
      x = pos0.x + rad*(double)sin(angle_radians);
      y = pos0.y + -1*rad*(double)cos(angle_radians);
      push = new Vector2d(x, y);
      sectorEnds.push_back(push);
   }


   pos = translateToScreen(sim->adv->projPath[0]->x, sim->adv->projPath[0]->y); 
   theta = 337.5;
   for(i = 0; i < 8; i++){
     s = new sector(theta, rad, pos);
     sectorList.push_back(s);
     theta = fmod(theta + 45.0, 360.0);
   }
   detChipLockPos();
}

void ui::selectChip(Vector2d p){
  std::vector<chip*>::iterator _i;
  std::vector<sector*>::iterator _j;
  int sectorFound;

  int found;

  // Test for grabbing visible chip
  for(_i = chips.begin(); _i != chips.end(); _i++){
    (*_i)->selectChip(p);
  }

  // Test if user is grabbing a chip from a sector that is already placed
  sectorFound = 0;
  for(_j = sectorList.begin(); _j != sectorList.end(); _j++){
    found = (*_j)->findChip(p);

    if(found > -1){
      for(_i = chips.begin(); _i != chips.end(); _i++){
        if( static_cast<int>((*_i)->getValue()) == found && (*_i)->getDrawChip() == false &&
            (*_i)->getChipSect() == sectorFound){
          (*_i)->setDrawChip(true);
          (*_i)->setPos(p);
          (*_i)->selectChip(p);
          (*_i)->setSector(-1);
          break;
        }
      }
    }
    sectorFound++;
  }
}

void ui::releaseChip(){
   std::vector<chip*>::iterator _i;
   Vector2d p, sectP, pos0;
   int initCS, finCS;

   p = translateToScreen(sim->adv->projPath[0]->x, sim->adv->projPath[0]->y); 
   for(_i = chips.begin(); _i != chips.end(); _i++){
      initCS = (*_i)->getChipSect();
      finCS = (*_i)->releaseChip(p, thetaList, rad);
      if(finCS == -1){
        (*_i)->setInitPos();
      }
      if(finCS != initCS){
         if(initCS != -1){
            sectorList[initCS]->removeChip(static_cast<int>((*_i)->getValue()));
            (*_i)->setDrawChip(true);
         }
         if(finCS != -1){
            // Find correct position to start looking
            sectorList[finCS]->addChip(static_cast<int>((*_i)->getValue()));
            (*_i)->setDrawChip(false);
         }
      }
   }
}

void ui::moveChip(Vector2d v){
   std::vector<chip*>::iterator _i;

   for(_i = chips.begin(); _i != chips.end(); _i++){
      (*_i)->moveChip(v);
   }
}

// Return 1 if all chips have been placed, 0 if not
int ui::chipsPlaced(){
   std::vector<chip*>::iterator _i;

   for(_i = chips.begin(); _i != chips.end(); _i++){
      if((*_i)->getChipSect() == -1){
         return 0;
      }
   }
   return 1;
}

void ui::reset(){
   std::vector<chip*>::iterator _ch;
   std::vector<sector*>::iterator _sl;

   cout << "Reseting everything" << endl;

   // Put all chips back
   for(_ch = chips.begin(); _ch != chips.end(); _ch++){
      (*_ch)->setInitPos();
      (*_ch)->setDrawChip(true);
   }

   // Remove all chip values from sectors
   for(_sl = sectorList.begin(); _sl != sectorList.end(); _sl++){
      (*_sl)->reset();
   }

   // Replace the slider
   slider.reset();
}

void ui::checkSliderPressed(Vector2d p){
   slider.checkSliderPressed(p);
}

void ui::moveSlider(Vector2d p){
   slider.moveSlider(p);
}

void ui::detChipLockPos(){
   std::vector<Vector2d*> chipSamplePos;
   std::vector<Vector2d*>::iterator _c;
   Vector2d * samp;
   Vector2d c, s0, s1, p, dif, u1, u2;
   double d;

   double angle_radians;
   double x, y;
   double theta, baseTheta;

   // Sample positions from hand placement of chips
   samp = new Vector2d(647, 209);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(707, 228);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(755, 254);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(638, 263);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(681, 275);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(723, 285);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(632, 302);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(663, 311);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(695, 320);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(633, 328);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(654, 335);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(674, 342);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(637, 350);
   chipSamplePos.push_back(samp);
   samp = new Vector2d(655, 357);
   chipSamplePos.push_back(samp);

   // Center and sector edges
   c.set(614.4, 425);
   s1.set(614.4, 715);
   baseTheta = 0.0;

   //cout << "BaseTheta : " << baseTheta << endl;

   angle_radians = 0.0*(double)M_PI/180.0;
   x = c.x + rad*(double)sin(angle_radians);
   y = c.y + -1*rad*(double)cos(angle_radians);
   s0.set(x, y);
   u2 = (s0-c).normalize();
   for(_c = chipSamplePos.begin(); _c != chipSamplePos.end(); _c++){
      u1 = (*(*_c)-c).normalize();
      theta = (acos(u1*u2)*180.0)/M_PI;

      if(u1.x < u2.x){
         theta = 360.0 - theta;
      }
      else if(u1.x == u2.x){
         if(u1.y == u2.y){
            theta = 0.0;
         }
         else{
            theta = 180.0;
         }
      }

      chipThetaList.push_back(theta-baseTheta);
      dif.set(abs((*_c)->x-c.x), abs((*_c)->y-c.y));
      d = sqrt(dif.x*dif.x + dif.y*dif.y);
      chipDistList.push_back(d);
   }

}

int ui::sectorValue(int i){
   std::vector<chip*>::iterator _c;
   int val = 0;

   for(_c = chips.begin(); _c != chips.end(); _c++){
      if( (*_c)->getChipSect() == i ){
         val += (*_c)->getValue();
      }
   }

   return val;
}

void ui::drawTargetArea(QGLWidget * g){
   std::vector<Vector2d*>::iterator _i;
   glColor4f(0.5, 0.0, 0.5, 1.0);
   glLineWidth(2);
   glBegin(GL_LINE_LOOP);
      for(_i = targetArea.begin(); _i != targetArea.end(); _i++){
         glVertex3f((*_i)->x,(*_i)->y,0.0); 
      }
   glEnd();

   drawSectors(g);
}

void ui::drawSectors(QGLWidget * g){
   std::vector<sector*>::iterator _s;
   for(_s = sectorList.begin(); _s != sectorList.end(); _s++){
     (*_s)->drawSectors(g);
   }
}

void ui::drawChips(){
   std::vector<chip*>::iterator _i;

   for(_i = chips.begin(); _i != chips.end(); _i++){
      (*_i)->drawChip();
   }
}

void ui::drawChipText(QGLWidget * g){
   Vector2d p;
   QString s;
   QFont f;

   int counter, i;

   f = QFont();
   f.setBold(true);
   glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

   counter = chips.size();
   for(i = 0; i < counter; i++){
      if(chips[i]->getDrawChip() == true){
        p = chips[i]->getPos();
        s = QString::number(chips[i]->getValue());

        // Value of 20
        if(i < 2){
           f.setPointSize(20);
           g->renderText(p.x-18, p.y+10, 0, s, f, 2000);
        }
        // Value of 10
        else if(i < 6){
           f.setPointSize(15);
           g->renderText(p.x-13, p.y+7, 0, s, f, 2000);
        }
        // Value of 5
        else if(i < 9){
           f.setPointSize(13);
           g->renderText(p.x-5, p.y+7, 0, s, f, 2000);
        }
        // Value of 1
        else{
           f.setPointSize(11);
           g->renderText(p.x-4, p.y+5, 0, s, f, 2000);
        }
     }
   }
}

void ui::drawChipBox(){
  double bx, by, tx, ty;

  bx = 815.0;
  by = 7.0;
  tx = 1017.0;
  ty = 183.0;

  glColor4f(0.0, 0.0, 0.0, 0.9);
  glLineWidth(3.0);
  glBegin(GL_LINES);
   glVertex3f(bx, by, 0.0); 
   glVertex3f(tx, by, 0.0); 

   glVertex3f(tx, ty, 0.0); 
   glVertex3f(bx, ty, 0.0); 

   glVertex3f(bx, ty, 0.0); 
   glVertex3f(bx, by, 0.0); 

   glVertex3f(tx, by, 0.0); 
   glVertex3f(tx, ty, 0.0); 
  glEnd();

  glColor4f(0.5, 0.5, 0.5, 0.9);
  glBegin(GL_POLYGON);
   glVertex3f(bx, by, 0.0); 
   glVertex3f(tx, by, 0.0); 
   glVertex3f(tx, ty, 0.0); 
   glVertex3f(bx, ty, 0.0); 
  glEnd();
}

void ui::drawSlider(){
  slider.drawSlider();
}

void ui::printSectors(){
   std::vector<Vector2d*>::iterator _i;
   Vector2d pos;

   cout << "Printing sectors..." << endl;
   pos = translateToScreen(sim->adv->projPath[0]->x, sim->adv->projPath[0]->y);

   cout << "Center: " << pos << endl;
   for(_i = sectorEnds.begin(); _i != sectorEnds.end()-6; _i++){
      cout << "   " << *(*_i) << endl;
   }

   cout << "Proj path: " << endl;
   for(unsigned int i = 0; i < sim->adv->projPath.size(); i++){
      cout << "   " << *(sim->adv->projPath[i]) << endl;
   }
   cout << "Finished! " << endl;
}
