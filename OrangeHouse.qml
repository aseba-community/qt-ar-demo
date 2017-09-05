import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0
import QtQuick 2.7

Entity {
    property matrix4x4 poseRelativeToWorldCenter

    components: [
        Transform {
            matrix: poseRelativeToWorldCenter.inverted()
        }
    ]

    Entity {
        components: [
            SceneLoader {
                source: "/models/orangehouse.qgltf"
            },
            // Transforms the 3D model to align it with the drawing on the marker.
            Transform {
                rotation: fromAxisAndAngle(Qt.vector3d(0,0,1), 180)
                translation: Qt.vector3d(0, 0, -0.038)
            }
        ]
    }
}
