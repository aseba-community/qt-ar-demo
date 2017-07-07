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

    Component.onCompleted:  tm.registerLinkNow(worldCenterFrame, cameraFrame, Qt.matrix4x4())

    Item {
        id: transformationUpdater

        function updateTransformations() {

        // coeff's
        var a = wCL.confidence; var b = oHL.confidence; var c = aHL.confidence
        var A = (a+b+c) == 0 ? 0 : a / (a+b+c)
        var B = (a+b+c) == 0 ? 0 : b / (a+b+c)
        var C = (a+b+c) == 0 ? 0 : c / (a+b+c)

        var minAB = Math.min(a, b)

        // raw transformations
        var Tcenter2cam_N_A = wCL.pose
        var Tcenter2cam_B_A = tm.getLinkBest(worldCenterFrame, cameraFrame)

        var Tcenter2orange_F = tm.getLinkBestCached(worldCenterFrame, orangeHouseFrame)
        var Tcenter2cam_N_B = oHL.pose.times(Tcenter2orange_F)
        var Tcenter2cam_B_B = tm.getLinkBest(orangeHouseFrame, cameraFrame).times(Tcenter2orange_F)

        var Tcenter2ada_F = tm.getLinkBestCached(worldCenterFrame, adaHouseFrame)
        var Tcenter2cam_N_C = aHL.pose.times(Tcenter2ada_F)
        var Tcenter2cam_B_C = tm.getLinkBest(adaHouseFrame, cameraFrame).times(Tcenter2ada_F)

        var Tcenter2orange_N = oHL.pose.inverted().times(Tcenter2cam_N_A)

        // rotation quaternion multiplied with their transposed
        var QR_Tcenter2cam_N_A = tm.toRotQuatProduct(Tcenter2cam_N_A)
        var QR_Tcenter2cam_B_A = tm.toRotQuatProduct(Tcenter2cam_B_A)

        var QR_Tcenter2cam_N_B = tm.toRotQuatProduct(Tcenter2cam_N_B)
        var QR_Tcenter2cam_B_B = tm.toRotQuatProduct(Tcenter2cam_B_B)

        var QR_Tcenter2cam_N_C = tm.toRotQuatProduct(Tcenter2cam_N_C)
        var QR_Tcenter2cam_B_C = tm.toRotQuatProduct(Tcenter2cam_B_C)

        var QR_Tcenter2orange_N = tm.toRotQuatProduct(Tcenter2orange_N)

        // calculation of avg. rotation quaternion
        var M = QR_Tcenter2cam_N_A.times(A*a).plus(
                QR_Tcenter2cam_B_A.times(A*(1-a))).plus(
                QR_Tcenter2cam_N_B.times(B*b)).plus(
                QR_Tcenter2cam_B_B.times(B*(1-b))).plus(
                QR_Tcenter2cam_N_C.times(C*c)).plus(
                QR_Tcenter2cam_B_C.times(C*(1-c)))

        var avgR = tm.getLargestEigenVecAsQuaternion(M)

        // calculation of avg. translation quaternion
        var QT_Tcenter2cam_N_A = tm.toTransVect(Tcenter2cam_N_A)
        var QT_Tcenter2cam_B_A = tm.toTransVect(Tcenter2cam_B_A)

        var QT_Tcenter2cam_N_B = tm.toTransVect(Tcenter2cam_N_B)
        var QT_Tcenter2cam_B_B = tm.toTransVect(Tcenter2cam_B_B)

        var QT_Tcenter2cam_N_C = tm.toTransVect(Tcenter2cam_N_C)
        var QT_Tcenter2cam_B_C = tm.toTransVect(Tcenter2cam_B_C)

        var avgT =  QT_Tcenter2cam_N_A.times(A*a).plus(
                    QT_Tcenter2cam_B_A.times(A*(1-a))).plus(
                    QT_Tcenter2cam_N_B.times(B*b)).plus(
                    QT_Tcenter2cam_B_B.times(B*(1-b))).plus(
                    QT_Tcenter2cam_N_C.times(C*c)).plus(
                    QT_Tcenter2cam_B_C.times(C*(1-c)))

        // camera transformation
        worldCenter2Camera = tm.toTransformationMatrix(avgR, avgT)


        M = QR_Tcenter2orange_N.times(minAB).plus(
                Tcenter2orange_F.times(1-minAB))

        avgR = tm.getLargestEigenVecAsQuaternion(M)

        var QT_Tcenter2orange_N = tm.toTransVect(Tcenter2orange_N)
        var QT_Tcenter2orange_F = tm.toTransVect(Tcenter2orange_F)

        avgT = QT_Tcenter2orange_N.times(minAB).plus(
               QT_Tcenter2orange_F.times(1-minAB))

        // orange house transformation
        center2OrangeHouse = tm.toTransformationMatrix(avgR, avgT)

        }
    }

    Connections {
        target: wCL
        onChanged: {
            //whenever the world center marker is good enough, transmem gets updated
            if(wCL.goodEnough){
                wCL.seenOnce = true
                tm.registerLinkNow(worldCenterFrame, cameraFrame, wCL.pose)
            }
            transformationUpdater.updateTransformations()
        }
    }

    Connections {
        target:  oHL
        onChanged: {
            // if the marker is good enough, we update the link between marker and the camera
            if(oHL.goodEnough){
                oHL.seenOnce = true
                tm.registerLinkNow(orangeHouseFrame, cameraFrame, oHL.pose)
            }
        }
    }

   Connections {
        target:  aHL
        onChanged: {
            if(aHL.goodEnough){
                aHL.seenOnce = true
                tm.registerLinkNow(adaHouseFrame, cameraFrame, aHL.pose)
            }
        }
    }
}
