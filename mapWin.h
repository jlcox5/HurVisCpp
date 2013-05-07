#ifndef MAPWIN_H_
#define MAPWIN_H

#include <QGLWidget>
#include <QImage>
#include <QMouseEvent>

#include "Matrix.h"

class mapWin : public QGLWidget{
   Q_OBJECT

   public:
      mapWin(QWidget * parent);
      ~mapWin();

   public slots:
      void update();
      void imageBind(GLubyte * toBind, int texNum, int height, int width);
      void imageOpen();

   protected:
      void mouseMoveEvent(QMouseEvent * e);
      void mousePressEvent(QMouseEvent * e);
      void mouseReleaseEvent(QMouseEvent * e);
      void keyPressEvent(QKeyEvent * k);
      void initializeGL();
      void resizeGL(int w, int h);
      void paintGL();

      void drawNext();

   private:
      GLuint texName;
      QTimer * timer;
      QImage img;
      Vector2d initPress;
      int completed;
      int step;

      // Button Variables - should be a separate class, but I'm rushing
      int buttonPressed;
      Vector2d butMax, butMin;
      Vector2d butMaxBord, butMinBord;
      Vector2d butTextPos;

      void drawCurPath();
      void checkButtonPress(double x, double y);
      void checkButtonRelease(double x, double y);

      void checkSliderPress(double x, double y);

      // Write out results
      void printStartExp();
      void printResults();
      void printStopExp();
};

#endif
