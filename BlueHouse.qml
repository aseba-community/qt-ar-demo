import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0
import QtQuick 2.7

Entity {
    property matrix4x4 poseRelativeToWorldCenter

    components: [
        Transform {
            matrix: t.inverted()
        }
    ]

    Entity {
        components: [
            SceneLoader {
                source: "/models/bluehouse.qgltf"
            }
        ]
    }
}
