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

import MarkerModel 1.0
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
		}
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

        property string cameraName : "cam"

        landmarks: [
			Landmark {
				id: worldCenterLandmark
                name: "world"
				fileName: ":/assets/markers/worldcenter.xml"
				property string icon: "assets/markers/worldcenter_tracker.png"
            },
			Landmark {
				id: orangeHouseLandmark
                name: "orangeHouse"
                fileName: ":/assets/markers/orangehouse.xml"
				property string icon: "assets/markers/orangehouse_tracker.png"
            },
            Landmark {
                id: adaHouseLandmark
                name: "adaHouse"
                fileName: ":/assets/markers/adahouse.xml"
                property string icon: "assets/markers/adahouse_tracker.png"
            }
		]
	}

	property rect cameraRect
	VideoOutput {
		id: videoOutput
		anchors.fill: parent
        //focus : visible
		source: camera
		filters: [vision]
		fillMode: VideoOutput.PreserveAspectCrop
		onContentRectChanged: cameraRect = mapNormalizedRectToItem(Qt.rect(0, 0, 1, 1));


	}

	Component.onCompleted: {

        // HACK
        markermodel.updateLinkNow(orangeHouseLandmark.name, vision.cameraName, Qt.matrix4x4(), 1)
        markermodel.updateLinkNow(adaHouseLandmark.name, vision.cameraName, Qt.matrix4x4(), 1)

		camera.start();
	}

	Component.onDestruction: {
		camera.stop();
	}

    MarkerModel {
        id: markermodel
    }

    Item {

        Connections {
            target: worldCenterLandmark
            onChanged: {
                markermodel.updateLinkNow(worldCenterLandmark.name, vision.cameraName, worldCenterLandmark.pose, 1.9)
                markermodel.updateModel()
            }
        }

        Connections {
            target: orangeHouseLandmark
            onChanged: {
                markermodel.updateLinkNow(orangeHouseLandmark.name, vision.cameraName, orangeHouseLandmark.pose, orangeHouseLandmark.confidence)
            }
        }

        Connections {
            target: adaHouseLandmark
            onChanged: {
                markermodel.updateLinkNow(adaHouseLandmark.name, vision.cameraName, adaHouseLandmark.pose, adaHouseLandmark.confidence)
            }
        }
    }

	Scene3d {
		anchors.fill: parent

        camera: markermodel.world2cam
        lens: vision.lens

        Entity {
           OrangeHouse {
                id: orangeHouse
                enabled: orangeHouseLandmark.found
                t: markermodel.world2orangeHouse
            }

           AdaHouse {
               id: adaHouse
               enabled: adaHouseLandmark.found
               t: markermodel.world2adaHouse
           }
        }

        WorldCenter {
			id: worldCenter
            enabled: worldCenterLandmark.found
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
