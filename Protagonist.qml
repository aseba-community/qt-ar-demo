import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0
import QtQuick 2.7

Entity {
    id: root
    components: [transform, mesh, material]

    PhongMaterial {
        id: material
        ambient: 'darkred'
    }

    Timer {
        running: true
        interval: 20
        onTriggered: transform.angle = (transform.angle + 1) % 360;
        repeat: true
    }

    Transform {
        id: transform
        property var angle : 0.0
        matrix: {
               var m = Qt.matrix4x4();
               m.rotate(angle, Qt.vector3d(0, 0, 1));
               m.translate(Qt.vector3d(0.25, 0, 0));
               return m;
           }
    }

    CuboidMesh {
        id: mesh
        xExtent: 0.1
        yExtent: 0.1
        zExtent: 0.1
    }
}
