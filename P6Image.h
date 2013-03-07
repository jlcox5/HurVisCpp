/* Jonathan Cox
*  CPSC 605
*
*  Header file for the P6 (RGB Binary) class
*/


#include "ImageFile.h"

class P6Image: public ImageFile {
   private:
      int maxValue;
   public:
      // Constructor takes open file stream
      P6Image(FILE * in);
      
      // One liners
      int getMaxValue(){ return maxValue; }  // Returns the maxValue of the color

      // Functions with some meat to them
      int findNextNumber(FILE * in);       // Finds the next digit in file header information
      int readData(FILE * in);             // Reads the actual image data
};

