import QtQuick 2.6
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.0
import QtQuick.Controls 2.0
import QtQuick.Controls.Material 2.0
import QtMultimedia 5.5
import Qt.labs.settings 1.0
import "qrc:/thymio-ar"
//import "qrc:/thymio-vpl2" as VPL2

ApplicationWindow {
	id: window
	title: qsTr("Thymio AR demo")
	visible: true
	width: 960
	height: 600

	header: ToolBar {
		RowLayout {
			anchors.fill: parent

			Repeater {
				model: (camera.cameraStatus === Camera.ActiveStatus && vision) ? vision.landmarks : 0
				delegate: ToolButton {
					contentItem: Item {
						Image {
							source: modelData.icon
							height: parent.height * 0.8
							width: parent.height * 0.8
						}
						ProgressBar {
							width: parent.width
							to: 0.5
							value: modelData.confidence
							anchors.bottom: parent.bottom
						}
					}
				}
			}
		}
	}

	Camera {
		id: camera

		focus {
			focusMode: Camera.FocusAuto
		}

		captureMode: Camera.CaptureViewfinder
		/*cameraState: Camera.LoadedState*/
		//deviceId: QtMultimedia.availableCameras[1].deviceId // hack to use second camera on laptop
	}

	Vision {
		id: vision
		landmarks: Landmark {
			id: landmark
			fileName: ":/assets/marker.xml"
			property string icon: "images/marker-312.png"
		}
	}

	property rect cameraRect
	VideoOutput {
		id: videoOutput
		anchors.fill: parent
		focus : visible
		source: camera
		filters: [vision]
		fillMode: VideoOutput.PreserveAspectCrop
		onContentRectChanged: cameraRect = mapNormalizedRectToItem(Qt.rect(0, 0, 1, 1));
	}

	Component.onCompleted: {
		camera.start();
		vision.calibrationRunning = true;
	}

	Component.onDestruction: {
		camera.stop();
	}

	Scene3d {
		anchors.fill: parent
		camera: landmark.pose
		lens: vision.lens
		WorldCenter {
			id: cave
			enabled: true
		}
	}

	// calibration rectangle
	Rectangle {
		visible: vision.calibrationRunning

		x: cameraRect.x  + (vision.calibrationRight ? cameraRect.width - cameraRect.height : 0)
		y: cameraRect.y
		height: cameraRect.height
		width: cameraRect.height
		opacity: 0.5

		transform: [
			Scale {
				xScale: 1 / cameraRect.height
				yScale: 1 / cameraRect.height
			},
			Matrix4x4 {
				matrix: vision.calibrationTransform
			},
			Scale {
				xScale: cameraRect.height
				yScale: cameraRect.height
			}
		]
	}

	// calibration progress bar
	ProgressBar {
		visible: vision.calibrationRunning

		width: parent.width / 3
		anchors.bottom: parent.bottom
		anchors.bottomMargin: parent.height / 3
		anchors.horizontalCenter: parent.horizontalCenter
		value: vision.calibrationProgress
	}
}
