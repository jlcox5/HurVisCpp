#ifndef MAGICK_SCREENSHOT_H_
#define MAGICK_SCREENSHOT_H_

//#include <Magick++.h>

// To compile, link with -lMagick++

//
// Will write a screenshot to the file name and path provided. 
// If no path, current directory is assumed. The file type is 
// inferred from the name of the image given. 
// E.g. "image.png" will automatically be written as a PNG file
// in the current directory.
//
inline void screenshot(const char* path) {
  // grab the current OpenGL display buffer and place it in frame
  // image channel vales will be stored as floats
  int	width = glutGet(GLUT_WINDOW_WIDTH);
  int	height = glutGet(GLUT_WINDOW_HEIGHT);
  float *frame = new float[4*width*height];
  /*glReadPixels(0,0,width,height,GL_RGBA,GL_FLOAT,frame);
  
  // copy the grabbed frame to an ImageMagick image object
  Magick::Image image;
  image.read(width, height, "RGBA", Magick::FloatPixel, frame);
  
  // the following is required because ImageMagick's image model starts at 
  // the top scanline, while OpenGL starts at the bottom scanline by default
  image.flip();
  
  // write the image out at high quality
  image.quality(90);
  image.write(path);*/
  
  delete frame;
}

#endif
