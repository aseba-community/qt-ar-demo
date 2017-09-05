TEMPLATE = app
QT += core quick widgets
SOURCES += \
    main.cpp \
    transmem/src/transmem.cpp \
    transmem/src/frameAndLink.cpp \
    transmem/src/stampedTransformation.cpp \
    transmem/src/transformationBuffer.cpp \
    transmem/src/graphMLWriter.cpp \
    markermodel.cpp

HEADERS += transmem/include/transmem/transmem.h \
    markermodel.h
INCLUDEPATH += transmem/include

RESOURCES += thymio-ar-demo.qrc
include(thymio-ar/thymio-ar.pri)
include(deployment.pri)

QT3D_MODELS += \
    assets/worldcenter-bare.dae \
    assets/worldcenter-vegetation.dae \
    assets/orangehouse.dae \
    assets/bluehouse.dae
QGLTF_PARAMS = -g
load(qgltf)

android {
    DISTFILES += \
        android/AndroidManifest.xml \
        android/res/values/libs.xml \
        android/build.gradle
    ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android
}
