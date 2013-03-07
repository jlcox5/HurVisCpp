/* Jonathan Cox
*  CPSC 605
*
*  This is the base class for all image formats.  The protected members are
*  used to store information that should be relevant to all images.  The
*  attribute imageData is used to store the data read from the file.
*  The pixMap attribute is used to store the information used to display the
*  image to the screen screen.  The functions given are used to control access
*  to the protected attributes.
*/


#ifndef IMAGEFILE_H
#define IMAGEFILE_H

#include <iostream>
#include <stdio.h>
#include <cctype>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

class ImageFile {
   protected:
      int width, height;
      GLubyte * imageData;
      unsigned int * pixMap;
   public:
      int getHeight(){ return height; }  // Returns height of picture
      int getWidth(){ return width; }  // Returns width of picture
      unsigned int * getPixMap(){ return pixMap; }
      GLubyte * getImageData(){ return imageData; }
};

#endif
