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
                // The adahousemodel is not available yet, therefore the model for the bluehouse is used instead.
                source: "/models/bluehouse.qgltf"
            },
            // Transform the model to align it with the drawing on the marker.
            Transform {
                scale3D: Qt.vector3d(0.45, 0.45, 0.45)
                translation: Qt.vector3d(0, 0, -0.038)
            }
        ]
    }
}
