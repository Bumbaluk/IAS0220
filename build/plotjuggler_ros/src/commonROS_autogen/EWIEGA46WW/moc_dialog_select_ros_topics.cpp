/****************************************************************************
** Meta object code from reading C++ file 'dialog_select_ros_topics.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/plotjuggler-ros-plugins/src/dialog_select_ros_topics.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dialog_select_ros_topics.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DialogSelectRosTopics_t {
    QByteArrayData data[12];
    char stringdata0[261];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DialogSelectRosTopics_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DialogSelectRosTopics_t qt_meta_stringdata_DialogSelectRosTopics = {
    {
QT_MOC_LITERAL(0, 0, 21), // "DialogSelectRosTopics"
QT_MOC_LITERAL(1, 22, 15), // "updateTopicList"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 40), // "std::vector<std::pair<QString..."
QT_MOC_LITERAL(4, 80, 10), // "topic_list"
QT_MOC_LITERAL(5, 91, 21), // "on_buttonBox_accepted"
QT_MOC_LITERAL(6, 113, 37), // "on_listRosTopics_itemSelectio..."
QT_MOC_LITERAL(7, 151, 26), // "on_maximumSizeHelp_pressed"
QT_MOC_LITERAL(8, 178, 29), // "on_lineEditFilter_textChanged"
QT_MOC_LITERAL(9, 208, 13), // "search_string"
QT_MOC_LITERAL(10, 222, 32), // "on_spinBoxArraySize_valueChanged"
QT_MOC_LITERAL(11, 255, 5) // "value"

    },
    "DialogSelectRosTopics\0updateTopicList\0"
    "\0std::vector<std::pair<QString,QString> >\0"
    "topic_list\0on_buttonBox_accepted\0"
    "on_listRosTopics_itemSelectionChanged\0"
    "on_maximumSizeHelp_pressed\0"
    "on_lineEditFilter_textChanged\0"
    "search_string\0on_spinBoxArraySize_valueChanged\0"
    "value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DialogSelectRosTopics[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x0a /* Public */,
       5,    0,   47,    2, 0x08 /* Private */,
       6,    0,   48,    2, 0x08 /* Private */,
       7,    0,   49,    2, 0x08 /* Private */,
       8,    1,   50,    2, 0x08 /* Private */,
      10,    1,   53,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    9,
    QMetaType::Void, QMetaType::Int,   11,

       0        // eod
};

void DialogSelectRosTopics::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DialogSelectRosTopics *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateTopicList((*reinterpret_cast< std::vector<std::pair<QString,QString> >(*)>(_a[1]))); break;
        case 1: _t->on_buttonBox_accepted(); break;
        case 2: _t->on_listRosTopics_itemSelectionChanged(); break;
        case 3: _t->on_maximumSizeHelp_pressed(); break;
        case 4: _t->on_lineEditFilter_textChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->on_spinBoxArraySize_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject DialogSelectRosTopics::staticMetaObject = { {
    QMetaObject::SuperData::link<QDialog::staticMetaObject>(),
    qt_meta_stringdata_DialogSelectRosTopics.data,
    qt_meta_data_DialogSelectRosTopics,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DialogSelectRosTopics::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DialogSelectRosTopics::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DialogSelectRosTopics.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int DialogSelectRosTopics::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
