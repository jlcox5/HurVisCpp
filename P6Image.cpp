/* Jonathan Cox
*  CPSC 605
*
*   This file houses the functions used by the P6Image (RGB Binary) class.  
*   These functions the following:
*
*   P6Image - Constructor which reads the header infomation and calls readData
*   readData - Parses through the binary data in the given file and stores it
*      in the classes pixMap attribute.  An alpha value of 255 is added.
*   findNextNumber - Finds the next number in the header file.  Used to find
*      height, width, and maxValue.
*/

#include <stdio.h>
#include <stdlib.h>
#include "P6Image.h"

P6Image::P6Image(FILE * in){
   //Reads the header information
   int ch;

   // Get white space immediately following
   ch = fgetc(in);
   ch = fgetc(in);
   ch = fgetc(in);
   //printf("%c\n", ch);
   if(!isspace(ch)){
      printf("Epic fail after P6\n");
         exit(1);
      }
      width = findNextNumber(in);
      // Reality check for width
      if(1 >= width and width >= 6000){
         printf("Width is out of normal bounds: 1-10000\n");
         exit(1);
      }
      // Reality check for height
      height = findNextNumber(in);
      if(1 >= height and height >= 6000){
         printf("Height is out of normal bounds: 1-10000\n");
         exit(1);
      }
      maxValue = findNextNumber(in);
      if(maxValue != 255 ){
         printf("Invalid max value\n");
         exit(1);
      }      
      printf("Width: %d, Height: %d, MaxValue:%d\n", width, height, maxValue);
      readData(in);
}

int P6Image::findNextNumber(FILE * in){
   int next;
   int ch;
   // Check for digit
   ch = fgetc(in);
   while(!isdigit(ch)){
      // If just more white space, get the rest
      if(isspace(ch)){
         ch = fgetc(in);
         while(isspace(ch)){
            ch = fgetc(in);
         }
      }
      // If comment, run through new line
      if(ch == '#'){
         while(ch != '\n'){
            ch = fgetc(in);
         }
      }
      // Nothing good can come of this.... (Not valid comment, whitespace, or digit)
      if(ch != '#' && !isspace(ch) && !isdigit(ch)){
         printf("File is corrupted.  Unable to parse header information.\n");
         exit(1);
      }
   }
   // This is the first digit
   next = ch-48;
   ch = fgetc(in);
   // This ch is a number, convert it to an int
   while(isdigit(ch)){
      next = next * 10;     // Move current digits up a place
      next = next+(ch-48);  // Add new digit to end
      ch = fgetc(in);
   }
   // Next space must be a whitespace
   if(!isspace(ch)){
      printf("Epic fail in findNextDigit - character after digit was not whitespace.\n");
      exit(1);
   }
   return next;
}


// Function name says it all
int P6Image::readData(FILE * in){
   imageData = new GLubyte[3 * width * height];
   pixMap = new unsigned int[width * height];
   int size, x, j, row;

   size = fread(imageData, 1, 3*width*height, in);
   if(size != 3*width*height){
      printf("Not enough image data. Size: %d,  Needed: %d\n", size, 3*width*height);
      exit(1);
   }

   j = width*height;
   for(x=0; x<width*height; x++){
      //printf("x: %d\n", x);
      row = height-(x/width);
      pixMap[(row*width-width)+(x%width)] = 255<<24 | imageData[x*3+2]<<16 | imageData[x*3+1]<<8 | imageData[x*3];
      j--;
   }
   return 0;
}
