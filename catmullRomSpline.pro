#    CSC 486 Assignment 1
#    Ryan Guy
#    V00484803

HEADERS       = glwidget.h \
                window.h \
    catmullromspline.h
SOURCES       = glwidget.cpp \
                main.cpp \
                window.cpp \
    catmullromspline.cpp
QT           += opengl widgets


contains(QT_CONFIG, opengles.) {
    contains(QT_CONFIG, angle): \
        warning("Qt was built with ANGLE, which provides only OpenGL ES 2.0 on top of DirectX 9.0c")
    error("This example requires Qt to be configured with -opengl desktop")
}
