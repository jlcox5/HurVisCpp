#include <iostream>

#include <QApplication>
#include <QMenuBar>
#include <QMenu>
#include <QVBoxLayout>

#include "window.h"
#include "mapWin.h"

window::window(QWidget * parent, Qt::WindowFlags f) : QWidget(parent, f)
{
   // Creating the opengl widget
   mapWin * m = new mapWin(this);
   
   // Menu system - Probably not needed....
   /*QMenu * file = new QMenu("File", this);
   file->addAction("Open...", m, SLOT(imageOpen()), Qt::CTRL+Qt::Key_O);
   file->addAction("Quit", QApplication::instance(), SLOT(quit()), Qt::CTRL+Qt::Key_Q); 

   QMenuBar * mb = new QMenuBar(this);
   mb->addSeparator();
   mb->addMenu(file);*/

   QVBoxLayout * vlayout = new QVBoxLayout(this);

   vlayout->setSpacing(0);
   vlayout->setMargin(0);

   //vlayout->setMenuBar(mb);
   vlayout->addWidget(m);
}

void window::keyPressEvent(QKeyEvent * k){
   if(k){
     std::cout << "Need to add interaction" << std::endl;
   }
}
