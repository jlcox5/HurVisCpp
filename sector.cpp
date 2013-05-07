/*
 * sector.cpp
 *
 *  Created on: Sep 25, 2011
 *  Jonathan Cox
 *  Clemson University
 */

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#  include <GL/glu.h>
#endif

#include <algorithm>
#include "sector.h"

bool sortChips(int i, int j){
  return i < j;
}

sector::sector(double theta, double r, Vector2d & pos0)
       :endPairs(), chipValues(), center() {
  double angle_radians;
  std::vector<Vector2d*> * pushV;
  double A, nr, x, y;
  Vector2d screenPos;
  Vector2d * push;

  theta1 = theta;
  theta2 = fmod(theta + 45.0, 360.0);

  //center = translateToScreen(pos0.x, pos0.y);
  center.set(pos0.x, pos0.y);
  A = M_PI*r*r;
  for(int i = 1; i < 102; i++){
    pushV = new std::vector<Vector2d*>();
    nr = sqrt((A*(static_cast<double>(i)/100.0))/M_PI);
    //nr = r*(static_cast<double>(i)/100.0);
    radius.push_back(nr);
    for(int j = 0; j < 46; j++){
      angle_radians = fmod((theta+static_cast<double>(j))*(double)M_PI/180.0, 360.0);
      x = pos0.x + nr*(double)sin(angle_radians);
      y = pos0.y + -1*nr*(double)cos(angle_radians);
      //screenPos = translateToScreen(x, y);
      //push = new Vector2d(screenPos.x, screenPos.y);
      push = new Vector2d(x, y);
      pushV->push_back(push);
    }
    endPairs.push_back(pushV);
    drawEnd.push_back(false);
  }
}

sector::~sector(){

}

void sector::drawSectors(QGLWidget * g){

  unsigned int i;
  int drawLast = 0;

  // Draw sector outline
  i = endPairs.size()-1;
  glColor4f(0.75, 0.0, 0.75, 0.75);
  glLineWidth(1);
  glBegin(GL_LINE_LOOP);
    glVertex3f(center.x, center.y, 0.0);
    for(unsigned int j = 0; j < endPairs[i]->size(); j++){
      glVertex3f(endPairs[i]->at(j)->x, endPairs[i]->at(j)->y, 0.0);
    }
  glEnd();

  // Draw area for chip values
  for(i = 0; i != endPairs.size(); i++){
    if(drawEnd[i] == true){
      glLineWidth(1);
      if( drawLast == 0 ){
        glColor4f(0.75, 0.0, 0.75, 0.5);
        glBegin(GL_POLYGON);
          glVertex3f(center.x, center.y, 0.0);
          for(unsigned int j = 0; j < endPairs[i]->size(); j++){
            glVertex3f(endPairs[i]->at(j)->x, endPairs[i]->at(j)->y, 0.0);
          }
        glEnd();
        glColor4f(0.75, 0.0, 0.75, 1.0);
        glBegin(GL_LINE_LOOP);
          glVertex3f(center.x, center.y, 0.0);
          for(unsigned int j = 0; j < endPairs[i]->size(); j++){
            glVertex3f(endPairs[i]->at(j)->x, endPairs[i]->at(j)->y, 0.0);
          }
        glEnd();
        drawLast = i;
      }
      else{
        for(unsigned int j = 0; j < endPairs[i]->size()-1; j++){
          glColor4f(0.75, 0.0, 0.75, 0.5);
          glBegin(GL_POLYGON);
            glVertex3f(endPairs[i]->at(j)->x, endPairs[i]->at(j)->y, 0.0);
            glVertex3f(endPairs[i]->at(j+1)->x, endPairs[i]->at(j+1)->y, 0.0);
            glVertex3f(endPairs[drawLast]->at(j+1)->x, endPairs[drawLast]->at(j+1)->y, 0.0);
            glVertex3f(endPairs[drawLast]->at(j)->x, endPairs[drawLast]->at(j)->y, 0.0);
          glEnd();
        }

        glColor4f(0.75, 0.0, 0.75, 1.0);
        glBegin(GL_LINE_LOOP);
        for(unsigned int j = 0; j < endPairs[i]->size(); j++){
          glVertex3f(endPairs[i]->at(j)->x, endPairs[i]->at(j)->y, 0.0);
        }
        for(int j = (int)(endPairs[drawLast]->size())-1; j >= 0; j--){
          glVertex3f(endPairs[drawLast]->at(j)->x, endPairs[drawLast]->at(j)->y, 0.0);
        }
        glEnd();
        drawLast = i;
        //cout << "DrawLast size: " << *endPairs[drawLast]->at(j) << endl;
      }
    }
  }
  drawValueText(g);
}

void sector::addChip(int add){
  int set = 0;
  int chipIndex;

  cout << "Start of chip values: " << endl;
  for(unsigned int i = 0; i < chipValues.size(); i++){
    set += chipValues[i];
    cout << "   " << chipValues[i] << endl;
  }
  
  set += add;
  cout << "   Final set: " << set << endl;
  chipValues.push_back(add);

  sort(chipValues.begin(), chipValues.end(), sortChips);
  for(unsigned int j = 0; j < drawEnd.size(); j++){
    drawEnd[j] = false;
  }
  chipIndex = 0;
  for(unsigned int j = 0; j < chipValues.size(); j++){
    chipIndex += chipValues[j];
    cout << "chipIndex: " << chipIndex << endl;
    drawEnd[chipIndex] = true;
  }
}

void sector::removeChip(int drop){
  std::vector<int>::iterator _c;

  for(_c = chipValues.begin(); _c != chipValues.end(); _c++){
    if((*_c) == drop){
      chipValues.erase(_c);
      break;
    }
  }
}

void sector::reset(){
  chipValues.clear();

  for(unsigned int i = 0; i < drawEnd.size(); i++){
    drawEnd[i] = false;
  }
}

int sector::findChip(Vector2d & p){
  double dist;
  int chipCount = 0;
  int ret, chipIndex;

  Vector2d u1, u2;
  double theta;

  u1 = (center-p).normalize();
   
  u2.set(0.0, 1.0);
  theta = (acos(u1*u2)*180.0)/M_PI;

  if(center.x > p.x){
    theta = 360.0 - theta;
  }
  else if(center.x == p.x){
    if(center.y == p.y){
      theta = 0.0;
    }
    else{
      theta = 180.0;
    }
  }

  if(theta1 > theta2){
    if(theta < theta1 && theta > theta2){
      cout << "r1" << endl;
      return -1;
    }
  }
  else if(theta < theta1 || theta >= theta2){
    cout << "r2" << endl;
    return -1;
  }

  dist = sqrt((p.x-center.x)*(p.x-center.x) + (p.y-center.y)*(p.y-center.y));
  cout << "dist: " << dist << endl;
  for(unsigned int i = 0; i < radius.size(); i++){
    if(drawEnd[i] == true && chipValues.size() > 0){
      cout << "   drawEnd is true... comparing: " << radius[i] << " and dist " << endl;
      if(radius[i] >= dist){
        // Take out chip value and rebuild drawEnd
        ret = chipValues[chipCount];
        removeChip(chipValues[chipCount]);
 
        for(unsigned int j = 0; j < drawEnd.size(); j++){
          drawEnd[j] = false;
        }
        cout << "   Reset draw end..." << endl;
        sort(chipValues.begin(), chipValues.end(), sortChips);
        chipIndex = 0;
        for(unsigned int j = 0; j < chipValues.size(); j++){
          chipIndex += chipValues[j];
          drawEnd[chipIndex] = true;
        }
        cout << "   Reset chips in sector..." << endl;
        cout << "r3" << endl;

        return ret;
      }
      chipCount++;
    }
  }

  cout << "r4" << endl;
  return -1; 
}

void sector::drawValueText(QGLWidget * g){
  Vector2d p0, p1, pF;
  QString s;
  QFont f;

  int sum = 0;

  // Find value to print
  for(unsigned int i = 0; i < chipValues.size(); i++){
    sum += chipValues[i];
  }

  s = QString::number(sum);
  f.setPointSize(20);

  // Find draw position

  glColor4f(0.0, 0.0, 0.5, 1.0);

  p0.set(endPairs[35]->at(0)->x, endPairs[35]->at(0)->y);
  p1.set(endPairs[35]->at(endPairs[35]->size()-1)->x, 
         endPairs[35]->at(endPairs[35]->size()-1)->y);

  pF = (p1-p0)*0.5;

  pF = p0 + pF;

  // Draw text
  g->renderText(pF.x-18, pF.y+10, 0, s, f, 2000);

}
