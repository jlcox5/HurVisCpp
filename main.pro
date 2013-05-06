CONFIG += warn_on release qt

SOURCES += advisory.cpp bin.cpp geoFunct.cpp gridStruct.cpp hurVis.cpp main.cpp               \
           Matrix.cpp P6Image.cpp path.cpp pathSegment.cpp posPoint.cpp predictedPath.cpp     \
           simulation.cpp Utility.cpp Vector.cpp velocityFunct.cpp window.cpp mapWin.cpp      \
           chip.cpp sector.cpp hurEyePoints.cpp slider.cpp
           
HEADERS += advisory.h bin.h geoFunct.h gridPoint.h gridStruct.h hurVis.h ImageFile.h          \
           Matrix.h P6Image.h path.h pathSegment.h posPoint.h predictedPath.h                 \
           simulation.h Utility.h Vector.h velocityFunct.h window.h mapWin.h chip.h sector.h  \
           hurEyePoints.h slider.h

TARGET = main

QT += opengl

INCLUDEPATH += .

TEMPLATE = app

OBJECTS_DIR = objs
MOC_DIR = mocs
