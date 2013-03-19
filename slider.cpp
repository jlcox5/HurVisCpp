/* Jonathan Cox
   Clemson University

   The slider allows users to control the points of the path that are shown.
*/

// Graphics Stuff
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#  include <GL/glu.h>
#endif

// Standard Libs

// Custom Libs
#include "slider.h"

Slider::Slider(): curPos(225,712), color(0.5, 0.5, 0.5, 0.9), sizeX(15), sizeY(30), 
                  sliderPressed(false){

}

void Slider::drawSlider(){
  
  glColor4f(color.x, color.y, color.z, color.w);
  glBegin(GL_POLYGON);
    glVertex3f(curPos.x, curPos.y, 0.0);
    glVertex3f(curPos.x+sizeX, curPos.y, 0.0);
    glVertex3f(curPos.x+sizeX, curPos.y+sizeY, 0.0);
    glVertex3f(curPos.x, curPos.y+sizeY, 0.0);
  glEnd();

  glLineWidth(2.0);
  glColor4f(0, 0, 0, color.w);
  glBegin(GL_LINE_LOOP);
    glVertex3f(curPos.x, curPos.y, 0.0);
    glVertex3f(curPos.x+sizeX, curPos.y, 0.0);
    glVertex3f(curPos.x+sizeX, curPos.y+sizeY, 0.0);
    glVertex3f(curPos.x, curPos.y+sizeY, 0.0);
  glEnd();

}

void Slider::checkSliderPressed(Vector2d p){
     if(p.x < curPos.x + sizeX && p.x > curPos.x){
      if(p.y < curPos.y + sizeY && p.y > curPos.y){
         cout << "Pressing the slider!" << endl;
         sliderPressed = true;
         return;
      }
   }
   sliderPressed = false;
   return;
}
