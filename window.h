#ifndef WINDOW_H_
#define WINDOW_H_

#include <QWidget>

class window : public QWidget{
   Q_OBJECT

   public:
      window(QWidget * parent = 0, Qt::WindowFlags f = 0);

      void keyPressEvent(QKeyEvent * k);

};

#endif
