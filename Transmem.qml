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

    property string worldCenterFrame: "wcf"
    property string orangeHouseFrame: "ohf"
    property string adaHouseFrame: "ahf"
    property string cameraFrame: "cf"

    TransMemQMLInterface {
        id: tm              // transmem object
    }

    Component.onCompleted: tm.registerLinkNow(worldCenterFrame, cameraFrame, Qt.matrix4x4())

    Connections {
        target: wCL
        onChanged: {

            //whenever the world center marker is activ, transmem gets updated
            if(wCL.active){
                wCL.seenOnce = true
                tm.registerLinkNow(worldCenterFrame, cameraFrame, wCL.pose)
            }

            // if both of the other markers are less or equal reliable than the world
            // center marker we dont use them for help
            if(wCL.confidence >= oHL.confidence && wCL.confidence >= aHL.confidence){
               worldCenter2Camera = tm.getInterpolation( tm.getLinkNow(worldCenterFrame, cameraFrame),
                                                         wCL.pose,
                                                         wCL.confidence / 0.7 )
            }
            // if one of the other marker is more reliable we use that one for support
            else {
                var markerFrame = oHL.confidence > aHL.confidence ? orangeHouseFrame : adaHouseFrame
                var pose = oHL.confidence > aHL.confidence ? oHL.pose : aHL.pose
                worldCenter2Camera = tm.getInterpolation( pose.times(tm.getLinkBestCached(worldCenterFrame, markerFrame)),
                                                          wCL.pose,
                                                          wCL.confidence / 0.7 )
            }
        }
    }

    Connections {
        target:  oHL
        onChanged: {
            // if the marker is active, we update the link between marker and the camera
            if(oHL.active){
                oHL.seenOnce = true
                tm.registerLinkNow(orangeHouseFrame, cameraFrame, oHL.pose)
            }

            var conf = oHL.confidence < wCL.confidence ? oHL.confidence : wCL.confidence

            center2OrangeHouse = tm.getInterpolation( tm.getLinkBestCached(worldCenterFrame, orangeHouseFrame),
                                                      tm.getLinkNow(worldCenterFrame, orangeHouseFrame),
                                                      conf / 0.7)
        }
    }

   Connections {
        target:  aHL
        onChanged: {
            if(aHL.active){
                aHL.seenOnce = true
                tm.registerLinkNow(adaHouseFrame, cameraFrame, aHL.pose)
            }
            var conf = aHL.confidence < wCL.confidence ? aHL.confidence : wCL.confidence

            center2AdaHouse = tm.getInterpolation( tm.getLinkBestCached(worldCenterFrame, adaHouseFrame),
                                                   tm.getLinkNow(worldCenterFrame, adaHouseFrame),
                                                   conf / 0.7)
        }
    }
}
