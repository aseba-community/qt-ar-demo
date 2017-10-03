import Qt3D.Core 2.0
import Qt3D.Render 2.0
import Qt3D.Extras 2.0

Entity {
    components: [
        SceneLoader {
            source: "/models/worldcenter-vegetation.qgltf"
        }
    ]
}
