import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

Entity {
	components: [
		SceneLoader {
			source: "/models/orangehouse.qgltf"
		},
		Transform {
			translation: Qt.vector3d(0, 0, -0.038) // the marker is on the top of the house
		}
	]
}
