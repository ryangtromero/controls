/****************************************************************************
** Meta object code from reading C++ file 'whirlybird_panel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../../../fsg/romer3/controls/whirlybird_ws/src/whirlybird_description/src/whirlybird_panel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'whirlybird_panel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_whirlybird_description__WhirlybirdPanel_t {
    QByteArrayData data[14];
    char stringdata0[172];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_whirlybird_description__WhirlybirdPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_whirlybird_description__WhirlybirdPanel_t qt_meta_stringdata_whirlybird_description__WhirlybirdPanel = {
    {
QT_MOC_LITERAL(0, 0, 39), // "whirlybird_description::Whirl..."
QT_MOC_LITERAL(1, 40, 8), // "setPitch"
QT_MOC_LITERAL(2, 49, 0), // ""
QT_MOC_LITERAL(3, 50, 9), // "pitch_deg"
QT_MOC_LITERAL(4, 60, 6), // "setYaw"
QT_MOC_LITERAL(5, 67, 7), // "yaw_deg"
QT_MOC_LITERAL(6, 75, 13), // "setPitchTopic"
QT_MOC_LITERAL(7, 89, 5), // "topic"
QT_MOC_LITERAL(8, 95, 11), // "setYawTopic"
QT_MOC_LITERAL(9, 107, 10), // "setEnabled"
QT_MOC_LITERAL(10, 118, 7), // "enabled"
QT_MOC_LITERAL(11, 126, 13), // "sendSetpoints"
QT_MOC_LITERAL(12, 140, 16), // "updatePitchTopic"
QT_MOC_LITERAL(13, 157, 14) // "updateYawTopic"

    },
    "whirlybird_description::WhirlybirdPanel\0"
    "setPitch\0\0pitch_deg\0setYaw\0yaw_deg\0"
    "setPitchTopic\0topic\0setYawTopic\0"
    "setEnabled\0enabled\0sendSetpoints\0"
    "updatePitchTopic\0updateYawTopic"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_whirlybird_description__WhirlybirdPanel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x0a /* Public */,
       4,    1,   57,    2, 0x0a /* Public */,
       6,    1,   60,    2, 0x0a /* Public */,
       8,    1,   63,    2, 0x0a /* Public */,
       9,    1,   66,    2, 0x0a /* Public */,
      11,    0,   69,    2, 0x09 /* Protected */,
      12,    0,   70,    2, 0x09 /* Protected */,
      13,    0,   71,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void whirlybird_description::WhirlybirdPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        WhirlybirdPanel *_t = static_cast<WhirlybirdPanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setPitch((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->setYaw((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->setPitchTopic((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->setYawTopic((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->setEnabled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->sendSetpoints(); break;
        case 6: _t->updatePitchTopic(); break;
        case 7: _t->updateYawTopic(); break;
        default: ;
        }
    }
}

const QMetaObject whirlybird_description::WhirlybirdPanel::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_whirlybird_description__WhirlybirdPanel.data,
      qt_meta_data_whirlybird_description__WhirlybirdPanel,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *whirlybird_description::WhirlybirdPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *whirlybird_description::WhirlybirdPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_whirlybird_description__WhirlybirdPanel.stringdata0))
        return static_cast<void*>(const_cast< WhirlybirdPanel*>(this));
    return rviz::Panel::qt_metacast(_clname);
}

int whirlybird_description::WhirlybirdPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
