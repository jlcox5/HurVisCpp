#############################################################################
# Makefile for building: main
# Generated by qmake (2.01a) (Qt 4.7.2) on: Tue May 7 10:54:17 2013
# Project:  main.pro
# Template: app
# Command: /usr/bin/qmake -unix -o Makefile main.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DQT_NO_DEBUG -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED
CFLAGS        = -pipe -O2 -D_REENTRANT -Wall -W $(DEFINES)
CXXFLAGS      = -pipe -O2 -D_REENTRANT -Wall -W $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++ -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4 -I. -I/usr/X11R6/include -Imocs
LINK          = g++
LFLAGS        = -Wl,-O1
LIBS          = $(SUBLIBS)  -L/usr/lib -L/usr/X11R6/lib -lQtOpenGL -lQtGui -lQtCore -lGLU -lGL -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = objs/

####### Files

SOURCES       = advisory.cpp \
		bin.cpp \
		geoFunct.cpp \
		gridStruct.cpp \
		hurVis.cpp \
		main.cpp \
		Matrix.cpp \
		P6Image.cpp \
		path.cpp \
		pathSegment.cpp \
		posPoint.cpp \
		predictedPath.cpp \
		simulation.cpp \
		Utility.cpp \
		Vector.cpp \
		velocityFunct.cpp \
		window.cpp \
		mapWin.cpp \
		chip.cpp \
		sector.cpp \
		hurEyePoints.cpp \
		slider.cpp \
		ui.cpp mocs/moc_window.cpp \
		mocs/moc_mapWin.cpp
OBJECTS       = objs/advisory.o \
		objs/bin.o \
		objs/geoFunct.o \
		objs/gridStruct.o \
		objs/hurVis.o \
		objs/main.o \
		objs/Matrix.o \
		objs/P6Image.o \
		objs/path.o \
		objs/pathSegment.o \
		objs/posPoint.o \
		objs/predictedPath.o \
		objs/simulation.o \
		objs/Utility.o \
		objs/Vector.o \
		objs/velocityFunct.o \
		objs/window.o \
		objs/mapWin.o \
		objs/chip.o \
		objs/sector.o \
		objs/hurEyePoints.o \
		objs/slider.o \
		objs/ui.o \
		objs/moc_window.o \
		objs/moc_mapWin.o
DIST          = /usr/share/qt4/mkspecs/common/g++.conf \
		/usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/opengl.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		main.pro
QMAKE_TARGET  = main
DESTDIR       = 
TARGET        = main

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: main.pro  /usr/share/qt4/mkspecs/linux-g++/qmake.conf /usr/share/qt4/mkspecs/common/g++.conf \
		/usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/opengl.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		/usr/lib/libQtOpenGL.prl \
		/usr/lib/libQtGui.prl \
		/usr/lib/libQtCore.prl
	$(QMAKE) -unix -o Makefile main.pro
/usr/share/qt4/mkspecs/common/g++.conf:
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/modules/qt_webkit_version.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/qt.prf:
/usr/share/qt4/mkspecs/features/unix/opengl.prf:
/usr/share/qt4/mkspecs/features/unix/thread.prf:
/usr/share/qt4/mkspecs/features/moc.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
/usr/lib/libQtOpenGL.prl:
/usr/lib/libQtGui.prl:
/usr/lib/libQtCore.prl:
qmake:  FORCE
	@$(QMAKE) -unix -o Makefile main.pro

dist: 
	@$(CHK_DIR_EXISTS) objs/main1.0.0 || $(MKDIR) objs/main1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) objs/main1.0.0/ && $(COPY_FILE) --parents advisory.h bin.h geoFunct.h gridPoint.h gridStruct.h hurVis.h ImageFile.h Matrix.h P6Image.h path.h pathSegment.h posPoint.h predictedPath.h simulation.h Utility.h Vector.h velocityFunct.h window.h mapWin.h chip.h sector.h hurEyePoints.h slider.h ui.h objs/main1.0.0/ && $(COPY_FILE) --parents advisory.cpp bin.cpp geoFunct.cpp gridStruct.cpp hurVis.cpp main.cpp Matrix.cpp P6Image.cpp path.cpp pathSegment.cpp posPoint.cpp predictedPath.cpp simulation.cpp Utility.cpp Vector.cpp velocityFunct.cpp window.cpp mapWin.cpp chip.cpp sector.cpp hurEyePoints.cpp slider.cpp ui.cpp objs/main1.0.0/ && (cd `dirname objs/main1.0.0` && $(TAR) main1.0.0.tar main1.0.0 && $(COMPRESS) main1.0.0.tar) && $(MOVE) `dirname objs/main1.0.0`/main1.0.0.tar.gz . && $(DEL_FILE) -r objs/main1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all: mocs/moc_window.cpp mocs/moc_mapWin.cpp
compiler_moc_header_clean:
	-$(DEL_FILE) mocs/moc_window.cpp mocs/moc_mapWin.cpp
mocs/moc_window.cpp: window.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) window.h -o mocs/moc_window.cpp

mocs/moc_mapWin.cpp: Matrix.h \
		Vector.h \
		Utility.h \
		mapWin.h
	/usr/bin/moc-qt4 $(DEFINES) $(INCPATH) mapWin.h -o mocs/moc_mapWin.cpp

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: compiler_moc_header_clean 

####### Compile

objs/advisory.o: advisory.cpp advisory.h \
		path.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		hurVis.h \
		geoFunct.h \
		velocityFunct.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h \
		simulation.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/advisory.o advisory.cpp

objs/bin.o: bin.cpp bin.h \
		gridPoint.h \
		Vector.h \
		Utility.h \
		simulation.h \
		advisory.h \
		path.h \
		posPoint.h \
		hurVis.h \
		geoFunct.h \
		velocityFunct.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/bin.o bin.cpp

objs/geoFunct.o: geoFunct.cpp geoFunct.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		simulation.h \
		advisory.h \
		path.h \
		hurVis.h \
		velocityFunct.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/geoFunct.o geoFunct.cpp

objs/gridStruct.o: gridStruct.cpp gridStruct.h \
		gridPoint.h \
		bin.h \
		Vector.h \
		Utility.h \
		geoFunct.h \
		posPoint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/gridStruct.o gridStruct.cpp

objs/hurVis.o: hurVis.cpp hurVis.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		geoFunct.h \
		velocityFunct.h \
		path.h \
		Matrix.h \
		advisory.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h \
		simulation.h \
		gridStruct.h \
		gridPoint.h \
		bin.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/hurVis.o hurVis.cpp

objs/main.o: main.cpp posPoint.h \
		Vector.h \
		Utility.h \
		hurVis.h \
		geoFunct.h \
		velocityFunct.h \
		advisory.h \
		path.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h \
		P6Image.h \
		ImageFile.h \
		Matrix.h \
		simulation.h \
		gridStruct.h \
		gridPoint.h \
		bin.h \
		window.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/main.o main.cpp

objs/Matrix.o: Matrix.cpp Matrix.h \
		Vector.h \
		Utility.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/Matrix.o Matrix.cpp

objs/P6Image.o: P6Image.cpp P6Image.h \
		ImageFile.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/P6Image.o P6Image.cpp

objs/path.o: path.cpp path.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		hurVis.h \
		geoFunct.h \
		velocityFunct.h \
		advisory.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h \
		simulation.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/path.o path.cpp

objs/pathSegment.o: pathSegment.cpp Vector.h \
		Utility.h \
		geoFunct.h \
		posPoint.h \
		pathSegment.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/pathSegment.o pathSegment.cpp

objs/posPoint.o: posPoint.cpp hurVis.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		geoFunct.h \
		velocityFunct.h \
		simulation.h \
		advisory.h \
		path.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/posPoint.o posPoint.cpp

objs/predictedPath.o: predictedPath.cpp predictedPath.h \
		Vector.h \
		Utility.h \
		pathSegment.h \
		geoFunct.h \
		posPoint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/predictedPath.o predictedPath.cpp

objs/simulation.o: simulation.cpp simulation.h \
		advisory.h \
		path.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		hurVis.h \
		geoFunct.h \
		velocityFunct.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/simulation.o simulation.cpp

objs/Utility.o: Utility.cpp Utility.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/Utility.o Utility.cpp

objs/Vector.o: Vector.cpp Vector.h \
		Utility.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/Vector.o Vector.cpp

objs/velocityFunct.o: velocityFunct.cpp hurVis.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		geoFunct.h \
		velocityFunct.h \
		path.h \
		Matrix.h \
		advisory.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/velocityFunct.o velocityFunct.cpp

objs/window.o: window.cpp window.h \
		mapWin.h \
		Matrix.h \
		Vector.h \
		Utility.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/window.o window.cpp

objs/mapWin.o: mapWin.cpp mapWin.h \
		Matrix.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		advisory.h \
		path.h \
		hurVis.h \
		geoFunct.h \
		velocityFunct.h \
		predictedPath.h \
		pathSegment.h \
		hurEyePoints.h \
		P6Image.h \
		ImageFile.h \
		simulation.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/mapWin.o mapWin.cpp

objs/chip.o: chip.cpp chip.h \
		Matrix.h \
		Vector.h \
		Utility.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/chip.o chip.cpp

objs/sector.o: sector.cpp sector.h \
		Vector.h \
		Utility.h \
		chip.h \
		Matrix.h \
		geoFunct.h \
		posPoint.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/sector.o sector.cpp

objs/hurEyePoints.o: hurEyePoints.cpp hurEyePoints.h \
		geoFunct.h \
		Vector.h \
		Utility.h \
		posPoint.h \
		predictedPath.h \
		pathSegment.h \
		advisory.h \
		path.h \
		hurVis.h \
		velocityFunct.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/hurEyePoints.o hurEyePoints.cpp

objs/slider.o: slider.cpp slider.h \
		Matrix.h \
		Vector.h \
		Utility.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/slider.o slider.cpp

objs/ui.o: ui.cpp ui.h \
		chip.h \
		Matrix.h \
		Vector.h \
		Utility.h \
		sector.h \
		geoFunct.h \
		posPoint.h \
		slider.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/ui.o ui.cpp

objs/moc_window.o: mocs/moc_window.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/moc_window.o mocs/moc_window.cpp

objs/moc_mapWin.o: mocs/moc_mapWin.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o objs/moc_mapWin.o mocs/moc_mapWin.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

