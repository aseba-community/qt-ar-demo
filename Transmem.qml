import QtQuick 2.0
import "qrc:/thymio-ar"
import TransMemQMLInterface 1.0
import QtSensors 5.0

Item {
    id: transmemWrapper

    property matrix4x4 worldCenter2Camera
    property matrix4x4 center2OrangeHouse
    property matrix4x4 center2AdaHouse

    property Landmark wCL   // world center landmark
    property Landmark oHL   // orange house landmark
    property Landmark aHL   // ada house landmark

    property RotationSensor rS  // rotation sensor of the device

    property string worldCenterFrame: "wcf"
    property string orangeHouseFrame: "ohf"
    property string adaHouseFrame: "ahf"
    property string cameraFrame: "cf"
    property string orientationSensorFrame: "osf"

    TransMemQMLInterface {
        id: tm              // transmem object
    }

    Component.onCompleted: tm.registerLinkNow(worldCenterFrame, cameraFrame, Qt.matrix4x4())

    /*
    Item {
        Timer {
            interval: 50; running: rS.active; repeat: true

            onTriggered: {
                var a1 = rS.reading.z
                var a2 = rS.reading.x
                var a3 = rS.reading.y

                var m = Qt.matrix4x4(   Math.cos(a1)*Math.cos(a3)-Math.sin(a1)*Math.sin(a2)*Math.sin(a3), -Math.cos(a2)*Math.sin(a1), Math.cos(a1)*Math.sin(a3)+Math.cos(a3)*Math.sin(a1)*Math.sin(a2), 0,
                                        Math.cos(a3)*Math.sin(a1)+Math.cos(a1)*Math.sin(a2)*Math.sin(a3),  Math.cos(a1)*Math.cos(a2), Math.sin(a1)*Math.sin(a3)-Math.cos(a1)*Math.cos(a3)*Math.sin(a2), 0,
                                                                              -Math.cos(a2)*Math.sin(a3),               Math.sin(a2),                                        Math.cos(a2)*Math.cos(a3), 0,
                                                                                                       0,                          0,                                                                0, 1 )

                tm.registerLinkNow(orientationSensorFrame, cameraFrame, m.inverted())

                worldCenter2Camera = m.inverted().times(tm.getLinkBestCached(worldCenterFrame, orientationSensorFrame))
                //console.log("update via rotation sensor")
            }
        }
    }
    */

    Connections {
        target: wCL
        onChanged: {
            /* whenever the world center marker is active, the camera pose can be directly set and
               transmem gets updated */
            if(wCL.active){
                wCL.seenOnce = true
                worldCenter2Camera = wCL.pose
                tm.registerLinkNow(worldCenterFrame, cameraFrame, wCL.pose)
            }
            /* if the world center marker is not visible (active), the position of the camera is calculated
               through one of the other visible markers */
            else {
                /* at the moment, just one of the two active marker is used to calculate the position of the camera
                   one could also average them or if more than two marker are available remove a possible outlier */
                if(oHL.active)
                    worldCenter2Camera = tm.getLinkNow(orangeHouseFrame, cameraFrame).times(tm.getLinkBestCached(worldCenterFrame, orangeHouseFrame))
                else if(aHL.active)
                    worldCenter2Camera = tm.getLinkNow(adaHouseFrame, cameraFrame).times(tm.getLinkBestCached(worldCenterFrame, adaHouseFrame))
            }
        }
    }

    Connections {
        target:  oHL
        onChanged: {
            /* if the marker is active, we update the link between marker and the camera */
            if(oHL.active){
                oHL.seenOnce = true
                tm.registerLinkNow(orangeHouseFrame, cameraFrame, oHL.pose)
            }
            /* if the marker and the the world center marker is active, we query for the link right now */
            if(wCL.active && oHL.active)
                center2OrangeHouse = tm.getLinkNow(worldCenterFrame, orangeHouseFrame)
            /* if not, we query for the best link back in time, asuming no one moves the marker */
            else
                center2OrangeHouse = tm.getLinkBestCached(worldCenterFrame, orangeHouseFrame)
         }
    }

   Connections {
        target:  aHL
        onChanged: {
            if(aHL.active){
                aHL.seenOnce = true
                tm.registerLinkNow(adaHouseFrame, cameraFrame, aHL.pose)
            }
            if(wCL.active && aHL.active)
                center2AdaHouse = tm.getLinkNow(worldCenterFrame, adaHouseFrame)
            else
                center2AdaHouse = tm.getLinkBestCached(worldCenterFrame, adaHouseFrame)
        }
    }
}
