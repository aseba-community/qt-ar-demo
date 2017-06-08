    TEMPLATE = app
    QT += core quick widgets
    SOURCES += main.cpp
    RESOURCES += thymio-ar-demo.qrc
    include(thymio-ar/thymio-ar.pri)
    include(deployment.pri)

    QT3D_MODELS += \
        assets/worldcenter-bare.dae \
        assets/worldcenter-vegetation.dae \
        assets/orangehouse.dae
    QGLTF_PARAMS = -g
    load(qgltf)

    android {
        DISTFILES += \
            android/AndroidManifest.xml \
            android/res/values/libs.xml \
            android/build.gradle
        ANDROID_PACKAGE_SOURCE_DIR = $$PWD/android
    }
