import QtQuick 2.7
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.0
import QtQuick.Controls 2.0
import QtQuick.Controls.Material 2.0
import QtQuick.Controls.Styles 1.4
import QtMultimedia 5.5
import Qt.labs.settings 1.0
import Qt3D.Core 2.0
import "qrc:/thymio-ar"
//import "qrc:/thymio-vpl2" as VPL2
import QtSensors 5.0

import MarkerModel 1.0
import Qt3D.Extras 2.0

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
            CheckBox {
                id: checkboxUseTransMem
                width: 300
                text: "Use TransMem"
                scale: 0.5
                font.bold: true
                font.pointSize: 30
                checked: true
            }
            CheckBox {
                id: checkboxShowProtagonist
                x: 400
                width: 50
                text: "Show protagonist"
                scale: 0.5
                font.bold: true
                font.pointSize: 30
                checked: true
            }
            ToolButton {
                contentItem: Image {
                    anchors.centerIn: parent
                    source: "icons/ic_filter_center_focus_black_24px.svg"
                }
                onClicked: vision.calibrationRunning = true;
            }
        }
    }

    Camera {
        id: camera

        // For the logitec c920 webcam the following viewfinder resolutions work:
        //  "640x480" / "1280x720" / "1600X986" / "1920x1080" */
        viewfinder.resolution: "1280x720"

        captureMode: Camera.CaptureViewfinder
        cameraState: Camera.LoadedState

        // Uncomment the following statement to use a second camera on a laptop (not the internal one)
        //deviceId: QtMultimedia.availableCameras[1].deviceId // hack to use second camera on laptop
    }

    /*
    // On some devices there occurs a "CameraBin error: Internal data flow error." when using am usb camera.
    // Using this hack restarts the camera after the error occured and bypasses the problem.
    Timer {
        running: true
        interval: 3000
        onTriggered: {
            camera.stop();
            camera.start();
        }
    }
    */

    // Timer to update the marker model.
    Timer {
        running: true
        interval: 20
        onTriggered: markermodel.updateModel();
        repeat: true
    }
    
    Vision {
        id: vision
        active: true

        // for NVidia Shield K1
        gyroscopeToCameraTransform: Qt.matrix4x4(
             0,  1,  0,  0,
            -1,  0,  0,  0,
             0,  0, -1,  0,
             0,  0,  0,  1
        )

        landmarks: [
            Landmark {
                id: worldCenterLandmark
                identifier: "world"
                fileName: ":/assets/markers/worldcenter.xml"
                property string icon: "assets/markers/worldcenter_tracker.png"
            },
            Landmark {
                id: orangeHouseLandmark
                identifier: "orangeHouse"
                fileName: ":/assets/markers/orangehouse.xml"
                property string icon: "assets/markers/orangehouse_tracker.png"
            },
            Landmark {
                id: adaHouseLandmark
                identifier:  "adaHouse"
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
        camera.start();
    }

    Component.onDestruction: {
        camera.stop();
    }

    MarkerModel {
        id: markermodel

        worldCenterMarker: worldCenterLandmark
        worldCenterRelativeMarkers: [orangeHouseLandmark, adaHouseLandmark]

        // Model specific parameter:
        useTransMem: checkboxUseTransMem.checked
    }

    Scene3d {
        anchors.fill: parent

        camera: worldCenterLandmark.relativePose
        lens: vision.lens

        Entity {
            OrangeHouse {
                id: orangeHouse
                poseRelativeToWorldCenter: orangeHouseLandmark.relativePose
                enabled: orangeHouseLandmark.visible
            }
            AdaHouse {
                id: adaHouse
                poseRelativeToWorldCenter: adaHouseLandmark.relativePose
                enabled: adaHouseLandmark.visible
            }

            // Very simplistic illustration of an "protagonist" walking through the scene.
            Protagonist {
                enabled: (worldCenterLandmark.visible || orangeHouseLandmark.visible || adaHouseLandmark.visible) && checkboxShowProtagonist.checked
            }
        }

        WorldCenter {
            id: worldCenter
            enabled: worldCenterLandmark.visible
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
