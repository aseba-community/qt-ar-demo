import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0
import QtQuick 2.7

Entity {
    property matrix4x4 t

    components: [
        Transform {
            matrix: t.inverted()
        }
    ]

    Entity {

        components: [
            SceneLoader {
                source: "/models/bluehouse.qgltf"       // at the moment i use the 3d model of the blue house for the ada house
            },
            Transform {
                scale3D: Qt.vector3d(0.45, 0.45, 0.45)
                translation: Qt.vector3d(0, 0, -0.038)
        }
        ]
    }
}
