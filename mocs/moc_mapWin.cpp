/****************************************************************************
** Meta object code from reading C++ file 'mapWin.h'
**
** Created: Tue Feb 26 14:05:23 2013
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mapWin.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mapWin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_mapWin[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x0a,
      44,   17,    7,    7, 0x0a,
      76,    7,    7,    7, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_mapWin[] = {
    "mapWin\0\0update()\0toBind,texNum,height,width\0"
    "imageBind(GLubyte*,int,int,int)\0"
    "imageOpen()\0"
};

const QMetaObject mapWin::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_mapWin,
      qt_meta_data_mapWin, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &mapWin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *mapWin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *mapWin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_mapWin))
        return static_cast<void*>(const_cast< mapWin*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int mapWin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: update(); break;
        case 1: imageBind((*reinterpret_cast< GLubyte*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 2: imageOpen(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
