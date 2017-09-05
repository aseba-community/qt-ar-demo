import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0
import QtQuick 2.7

// Very simple object to illustrate a protagonist.
Entity {

    components: [protagonistTransformation, protagonistMesh, protagonistMaterial]

    PhongMaterial {
        id: protagonistMaterial
        ambient: 'darkred'
    }

    SphereMesh {
        id: protagonistMesh
        radius: 0.05
    }

    Transform {
        id: protagonistTransformation
        property var angle : 0.0
        matrix: {
               var m = Qt.matrix4x4();
               m.rotate(angle, Qt.vector3d(0, 0, 1));
               m.translate(Qt.vector3d(0.25, 0, 0));
               return m;
           }

        // Updates the position of the protagonist.
        Timer {
            running: true
            interval: 20
            onTriggered: parent.angle = (parent.angle + 1) % 360;
            repeat: true
        }
    }
}
