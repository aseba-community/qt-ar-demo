import QtQuick 2.7
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.0
import QtQuick.Controls 2.0
import QtQuick.Controls.Material 2.0
import QtMultimedia 5.5
import Qt.labs.settings 1.0
import Qt3D.Core 2.0
import "qrc:/thymio-ar"
//import "qrc:/thymio-vpl2" as VPL2

import QtSensors 5.0

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
							x: parent.height * 0.1
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

			Item {
				 Layout.fillWidth: true
			}

			ToolButton {
				contentItem: Image {
					anchors.centerIn: parent
					source: "icons/ic_filter_center_focus_black_24px.svg"
				}
				onClicked: vision.calibrationRunning = true;
			}

            ToolButton {
               contentItem: Image {
                    anchors.centerIn: parent
                    source: "icons/ic_reset_marker_black_24px.svg"
               }
               onClicked: {
                    for(var i = 0; i < vision.landmarks.length; i++){
                        vision.landmarks[i].seenOnce = false
                    }
               }
            }
            ToolButton {
                contentItem: Image {
                    id: buttonImage
                    anchors.centerIn: parent
                    source : "icons/ic_rotation_support_off_24px.svg"
                }
                onClicked: {
                    console.log("pressed")
                    vision.active = !vision.active
                    buttonImage.source = rotationSensor.active ? "icons/ic_rotation_support_on_24px.svg" : "icons/ic_rotation_support_off_24px.svg"
                }
            }
		}
	}

    Transmem {
        id: transmem
        // marker
        wCL: worldCenterLandmark
        oHL: orangeHouseLandmark
        aHL: adaHouseLandmark
        // rotation sensor
        rS: rotationSensor

    }

    RotationSensor {
        id: rotationSensor
        active: false
    }

    Camera {
		id: camera

		focus {
			focusMode: Camera.FocusAuto
		}

		captureMode: Camera.CaptureViewfinder
		cameraState: Camera.LoadedState
		//deviceId: QtMultimedia.availableCameras[1].deviceId // hack to use second camera on laptop
	}

    // HACK
    Timer {
        running: true
        interval: 3000
        onTriggered: {
            camera.stop();
            camera.start();
        }
    }

	Vision {
		id: vision
        active: true
        landmarks: [
			Landmark {
				id: worldCenterLandmark
				fileName: ":/assets/markers/worldcenter.xml"
				property string icon: "assets/markers/worldcenter_tracker.png"
            },
			Landmark {
				id: orangeHouseLandmark
				fileName: ":/assets/markers/orangehouse.xml"
				property string icon: "assets/markers/orangehouse_tracker.png"
            },
            Landmark {
                id: adaHouseLandmark
                fileName: ":/assets/markers/adahouse.xml"
                property string icon: "assets/markers/adahouse_tracker.png"
            }
		]
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
	}

	Component.onDestruction: {
		camera.stop();
	}

	Scene3d {
		anchors.fill: parent

        camera: transmem.worldCenter2Camera

        lens: vision.lens

        Entity {
           OrangeHouse {
                id: orangeHouse
                enabled: orangeHouseLandmark.seenOnce && vision.leastOneMarkerActive
                t: transmem.center2OrangeHouse
            }

           AdaHouse {
               id: adaHouse
               enabled: adaHouseLandmark.seenOnce && vision.leastOneMarkerActive
               t: transmem.center2AdaHouse
           }
        }

        WorldCenter {
			id: worldCenter
            enabled: worldCenterLandmark.seenOnce && vision.leastOneMarkerActive
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
