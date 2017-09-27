#include "markermodel.h"

/*! \fn QMatrix4x4 rotAndTransPair2Matrix(const rotAndTransPair &qp)
 * \brief Creates a 4x4 matrix from the rotAndTransPair \a qp (\see rotAndTransPair).
 * \param qp RotAndTransPair describing a transformation through a quaternion and a vector.
 * \return A 4x4 matrix representing the mapping equal to the transformation described by \a qp.
 */
QMatrix4x4 rotAndTransPair2Matrix(const rotAndTransPair &qp){

    QMatrix4x4 ret(qp.first.toRotationMatrix());
    ret(0,3) = qp.second.x();   ret(1,3) = qp.second.y(); ret(2,3) = qp.second.z();
    return ret;

}

/*! \fn rotAndTransPair matrix2rotAndTransPair(const QMatrix4x4 &m)
 * \brief Separates the transformation encoded in the 4x4 matrix \a m into a rotation quaternion
 * and a translation vector which are returned in a rotAndTransPair object (\see rotAndTransPair).
 * \param m Encodes a transformation.
 * \return A rotAndTransPair encoding the transformation described through \a m.
 */
rotAndTransPair matrix2rotAndTransPair(const QMatrix4x4 &m){

    float data[]{ m(0,0),m(0,1),m(0,2),
                m(1,0),m(1,1),m(1,2),
                m(2,0),m(2,1),m(2,2)
    };

    QMatrix3x3 rM(data);

    return rotAndTransPair{QQuaternion::fromRotationMatrix(rM), QVector3D(m(0,3), m(1,3), m(2,3))};
}

/*! \QVector4D compareRotAndTransPair(const rotAndTransPair &qp1, const rotAndTransPair &qp2)
 * \brief Compares the two transformations encoded through \a qp1 and \a qp2. As a result of the
 * comparison a vector is returned. Each component of this vector describes the result of one of four comparisons:
 * - x component: Ratio between the length of the two translation vectors mapped to [0,2]
 * - y component: Distance between the two normalized translation vectors.
 * - z component: Distance between a point perpendicular to the rotation axis with
 *                length one and this point rotated by the angle difference
 * - w component: Distance between the two normalized rotation vectors.
 * For the comparison each of the two transformations is internally splitted up into its normalized rotation axis, the
 * corresponding angle and into the normalized translation vector with a corrensponding scalar magnitude.
 * \return The 4D vector encoding the difference of two transformations in its four components x,y,z and w.
 */
QVector4D compareRotAndTransPair(const rotAndTransPair &qp1, const rotAndTransPair &qp2){

    const double toRad = M_PI/180;

    // We first split each transformation in a translation vector vTra,
    // a rotation axis vRotA and an angle fAngl.
    QVector3D vTra1 = qp1.second,
            vTra2 = qp2.second;

    QVector3D vRotA1, vRotA2; float fAngl1, fAngl2;

    qp1.first.getAxisAndAngle(&vRotA1, &fAngl1);
    qp2.first.getAxisAndAngle(&vRotA2, &fAngl2);

    // Calculate ratio between the length of the two translation vectors and map them to the interval [0,2].
    double ratioLenT;
    double lenT1 = vTra1.length(), lenT2 = vTra2.length();
    if(lenT1 == 0 && lenT2 == 0)
        ratioLenT = 0;
    else if(lenT1 == 0 || lenT2 == 0)
        ratioLenT = 2;
    else
        ratioLenT = lenT2 > lenT1 ? (1-(lenT1/lenT2))*2. : (1-(lenT2/lenT1))*2.;

    // Calculate the distance between the two normalized translation vectors.
    vTra1.normalize(); vTra2.normalize();
    double distanceNormalT = fabs((vTra1 - vTra2).length());

    // Make sure the two rotation axes are comparable.
    if(vRotA1.z() < 0 ){ vRotA1 *= -1; fAngl1 = 360 - fAngl1; }
    if(vRotA2.z() < 0 ){ vRotA2 *= -1; fAngl2 = 360 - fAngl2; }

    // Calculate the distance between a point perpendicular to the rotation axis
    // with length one and this point rotated by the angle difference.
    double distanceNormalPointRSameDir = sqrt(2*(1-cos((fAngl1-fAngl2)*toRad)));
    double distanceNormalPointROppositeDir = sqrt(2*(1-cos((fAngl1+fAngl2)*toRad)));

    // Calculate the distance between the two rotation vectors.
    vRotA1.normalize(); vRotA2.normalize();
    double distRotAxesSameDir = fabs((vRotA1 - vRotA2).length());
    double distRotAxesOppositeDir = fabs(((vRotA1) - (vRotA2*-1)).length());


    double distanceNormalPointR, distRotAxesZ;
    if(distRotAxesSameDir <= distRotAxesOppositeDir){
        distRotAxesZ = distRotAxesSameDir;
        distanceNormalPointR = distanceNormalPointRSameDir;
    }
    else{
        distRotAxesZ = distRotAxesOppositeDir;
        distanceNormalPointR = distanceNormalPointROppositeDir;
    }

    // Return a vector encoding the "difference" between two transformations.
    // For equal transformation the method returns QVector4D(0,0,0,0).
    //x             //y                 //z                     //w
    return QVector4D(   ratioLenT,     distanceNormalT,    distanceNormalPointR,   distRotAxesZ);

}

/*! \fn QQuaternion avgAndNormalizeQuaternions(const QQuaternion &q1, const QQuaternion &q2)
 * \brief Creates a new quaternion by averaging the two quaternions \a q1 and \a q2. The created
 * quaternion is normalized before it is returned.
 */
QQuaternion avgAndNormalizeQuaternions(const QQuaternion &q1, const QQuaternion &q2){

    QQuaternion ret = QQuaternion( (q1.scalar() + q2.scalar()) / 2.,
                                   (q1.x() + q2.x()) / 2.,
                                   (q1.y() + q2.y()) / 2.,
                                   (q1.z() + q2.z()) / 2. );

    return ret.normalized();
}

/*! \fn QVector3D avgVector3D(const QVector3D &v1, const QVector3D &v2)
 * \brief Creates a new vector by averaging the two vectors \a v1 and \a v2 component-wise.
 */
QVector3D avgVector3D(const QVector3D &v1, const QVector3D &v2){

    QVector3D ret = QVector3D((v1.x() + v2.x()) * 0.5,
                              (v1.y() + v2.y()) * 0.5,
                              (v1.z() + v2.z()) * 0.5
                              );
    return ret;
}

/*! \fn bool equalTransformation(const rotAndTransPair &qp1, const rotAndTransPair &qp2,
 *                               const QVector4D &thTransformationEquality)
 * \brief Determines whether the two transformations encoded in \a qp1 and \a qp2 are equal. The two transformations
 * are considered to be equal if the difference vector calculated with compareRotAndTransPair(..) is component-wise
 * smaller than the threshold \a thTransformationEquality.
 */
bool equalTransformation(const rotAndTransPair &qp1, const rotAndTransPair &qp2,
                         const QVector4D &thTransformationEquality = QVector4D(0.2, 0.2, 0.2, 0.2)){
    // thTransformationEquality:
    // Parameter to decide if to transforamtion are equal. ratioLenT, distanceNormalT, distanceNormalPointR, distanceNormalR

    QVector4D diff = compareRotAndTransPair(qp1, qp2);

    return diff.x() < thTransformationEquality.x() && diff.y() < thTransformationEquality.y() &&
            diff.z() < thTransformationEquality.z() && diff.w() < thTransformationEquality.w();
}

// Constructor  for the marker model, just need to specify it when we use the monitoring functionality.
#ifdef USE_MONITORING_FUNCTIONALITY
MarkerModel::MarkerModel(){

    // Necessary to allow connections to marker model monitor
    qRegisterMetaType<Timestamp>(); qRegisterMetaType<rotAndTransPair>(); qRegisterMetaType<std::string>();

    // Create a new monitor object
    MarkerModelMonitor* monitor = new MarkerModelMonitor;
    monitor->moveToThread(&monitorThread);

    connect(this, &MarkerModel::linkUpdate, monitor, &MarkerModelMonitor::monitorLinkUpdate);
    connect(this, &MarkerModel::transformationUpdate, monitor, &MarkerModelMonitor::monitorTransformationUpdate);

    connect(this, &MarkerModel::startMonitor, monitor, &MarkerModelMonitor::startMonitoring);
    connect(this, &MarkerModel::stopMonitor, monitor, &MarkerModelMonitor::stopMonitoring);

    connect(this, &MarkerModel::registerLinkUpdateToMonitor, monitor, &MarkerModelMonitor::registerLinkUpdateToMonitor);
    connect(this, &MarkerModel::registerTransformationToMonitor, monitor, &MarkerModelMonitor::registerTransformationToMonitor);

    monitorThread.start();
}
#endif

/* STWC = StampedTransformationWithConfidence) */
// Multiplies two STWC objects
StampedTransformationWithConfidence MarkerModel::multiplySTWC(const StampedTransformationWithConfidence &lhs, const StampedTransformationWithConfidence &rhs){

    StampedTransformationWithConfidence ret;
    ret.time = lhs.time;

    ret.rotation = lhs.rotation * rhs.rotation;
    ret.translation = lhs.rotation * rhs.translation;
    ret.translation = ret.translation + lhs.translation;

    ret.averageLinkConfidence = std::max(lhs.averageLinkConfidence, rhs.averageLinkConfidence);
    ret.maxDistanceToEntry = std::max(lhs.maxDistanceToEntry, rhs.maxDistanceToEntry);

    return ret;
}

// Averaging two STWC objects if the encode an "equal" transformation.
StampedTransformationWithConfidence MarkerModel::averageSTWCifEqual(StampedTransformationWithConfidence &lhs, StampedTransformationWithConfidence &rhs){

    bool encodeEqualTransformation = equalTransformation(rotAndTransPair{lhs.rotation, lhs.translation}, rotAndTransPair{rhs.rotation, rhs.translation});

    if(!encodeEqualTransformation)
        return lhs;

    StampedTransformationWithConfidence ret;

    ret.rotation = avgAndNormalizeQuaternions(lhs.rotation, rhs.rotation);
    ret.translation = avgVector3D(lhs.translation, rhs.translation);
    ret.averageLinkConfidence = (lhs.averageLinkConfidence + rhs.averageLinkConfidence ) / 2;
    ret.maxDistanceToEntry = lhs.maxDistanceToEntry;
    ret.time = lhs.time;

    return ret;
}

// Invert the tranformation encoded in a STWC object.
StampedTransformationWithConfidence MarkerModel::invertSTCW(StampedTransformationWithConfidence &lhs){

    lhs.rotation = lhs.rotation.inverted();
    lhs.translation = -(lhs.rotation*lhs.translation);

    return lhs;
}

// Slot gets called whenever the pose of a marker determined by the tracker was updated
void MarkerModel::markerPositionUpdated(){

    Timestamp tsNow = std::chrono::high_resolution_clock::now();

    // Determine which landmark was the sender of the signal
    QObject *sender = QObject::sender();
    Landmark *senderLandmark = qobject_cast<Landmark*>(sender);

    QMatrix4x4 pose = senderLandmark->pose();
    double confidence = senderLandmark->confidence();
    std::string markerID = (senderLandmark->identifier).toStdString();

    // If the marker model is ran without transmem we don't store anything in transmem
    if(useTransMem){
        if(confidence >= thConfUpdate)
            registerLink(markerID, camID, tsNow, pose, confidence);
        else
            try { updateLinkConfidence(markerID, camID, confidence); }
        catch(NoSuchLinkFoundException e) { /* Quality can just be updated if there was already a successful update. */ }
    }

#ifdef USE_MONITORING_FUNCTIONALITY
    emit linkUpdate(markerID, camID, tsNow, matrix2rotAndTransPair(pose), confidence);
#endif
}

// Whenever this function is called, the relative transformation for all marker are update with the best information available
void MarkerModel::updateModel(){

    // Make sure there is a world center marker.
    if(!worldCenterMarker){
        qWarning() << "No world center marker available. Returning.";
        return;
    }

    // At what time do we want to update the model?
    Timestamp tsNow = std::chrono::high_resolution_clock::now();
    // Transformation from the world center to the camera.
    StampedTransformationWithConfidence world2camNow;

    if(useTransMem){
        try{ world2camNow = getLink(worldID, camID, tsNow); }
        catch(NoSuchLinkFoundException){ /* world center marker not seen yet. */ }
    }
    else{
        rotAndTransPair rotAndTrans = matrix2rotAndTransPair(worldCenterMarker->pose());
        world2camNow.rotation = rotAndTrans.first;
        world2camNow.translation = rotAndTrans.second;
        world2camNow.averageLinkConfidence = worldCenterMarker->confidence();
        world2camNow.maxDistanceToEntry = 0.;
    }

    // The world center marker is not visible if the confidence is to bad.
    worldCenterMarker->visible = !(world2camNow.averageLinkConfidence < thConfVisible);
    // The world center marker is also not visible if the last update happened too long ago.
    if(world2camNow.maxDistanceToEntry > thLastUpdate)
        worldCenterMarker->visible = false;
    emit worldCenterMarker->visibilityUpdated();

    unsigned int numberOfRelativeMarker = relativeMarkers.count();
    // If no additional marker are available, we can't do that much..
    if(numberOfRelativeMarker < 1){

        worldCenterMarker->relativePose = rotAndTransPair2Matrix(rotAndTransPair{world2camNow.rotation, world2camNow.translation});

        emit worldCenterMarker->relativePoseUpdated();
        emit worldCenterMarker->visibilityUpdated();

        return;
    }

    if(useTransMem){
        // Storage for all relative marker which can be updated.
        QVector<Landmark*> updatableMarker = QVector<Landmark*>();
        // Storage for all transformations mapping from a relative marker to the camera.
        QVector<StampedTransformationWithConfidence>  cam2relativeMarkersNow = QVector<StampedTransformationWithConfidence>();
        // Storage for all best transformations mapping from the world center to a relative marker.
        QVector<StampedTransformationWithConfidence> world2relativeMarkersFix = QVector<StampedTransformationWithConfidence>();

        StampedTransformationWithConfidence relativeMarker2camNow, world2relativeMarkerFix, world2camOption;
        std::string markerID;
        // First check for every marker but the world center marker if it is visible.
        for(Landmark* relativeMarker : relativeMarkers){

            // Just make sure we dont run into a null pointer here.
            if(relativeMarker == nullptr)
                continue;

            markerID = (relativeMarker->identifier).toStdString();

            try{ relativeMarker2camNow = getLink(markerID, camID, tsNow); }
            catch(NoSuchLinkFoundException){
                continue; /* no link registered yet */ }

            try{ world2relativeMarkerFix = getBestLink(worldID, markerID); }
            catch(NoSuchLinkFoundException){ continue; /* no link registered yet */ }

            // If the current confidence is to low we assume the marker is not visible.
            relativeMarker->visible = !(relativeMarker2camNow.averageLinkConfidence < thConfVisible);

            // We just consider the marker for further calculations if there was recently was an update
            // and the fix transformation is also good enough, otherwise we set it to invisible
            if( relativeMarker2camNow.maxDistanceToEntry > thLastUpdate ||
                    world2relativeMarkerFix.maxDistanceToEntry > thFixUpdate) {

                relativeMarker->visible = false;
                relativeMarker->visibilityUpdated();
                continue;
            }

            // Calculate optional mapping from world to cam
            world2camOption = multiplySTWC(relativeMarker2camNow, world2relativeMarkerFix);

            if(world2camNow.maxDistanceToEntry > thLastUpdate)
                world2camNow = world2camOption;
            else
                // Average the mappings to reduce scatter?
                world2camNow = averageSTWCifEqual(world2camNow, world2camOption);

            updatableMarker.push_back(relativeMarker);
            cam2relativeMarkersNow.push_back(invertSTCW(relativeMarker2camNow));
            world2relativeMarkersFix.push_back(world2relativeMarkerFix);

        }

        StampedTransformationWithConfidence cam2relativeMarkerNow, world2relativeMarkerNow;
        // The transformation for every marker but the world center marker which is visible can be updated.
        for(int indx = 0; indx < updatableMarker.count(); indx++){

            Landmark* updatedMarker = updatableMarker.at(indx);

            cam2relativeMarkerNow = cam2relativeMarkersNow.at(indx);
            world2relativeMarkerFix = world2relativeMarkersFix.at(indx);

            world2relativeMarkerNow = multiplySTWC(cam2relativeMarkerNow, world2camNow);
            world2relativeMarkerNow = averageSTWCifEqual(world2relativeMarkerNow, world2relativeMarkerFix);

            updatedMarker->relativePose = rotAndTransPair2Matrix(rotAndTransPair{world2relativeMarkerNow.rotation, world2relativeMarkerNow.translation});

            emit updatedMarker->relativePoseUpdated();
            emit updatedMarker->visibilityUpdated();
        }
    }
    // Without the use of TransMem.
    else {

        StampedTransformationWithConfidence relativeMarker2camNow;
        for(Landmark* relativeMarker : relativeMarkers){

            rotAndTransPair rotAndTrans = matrix2rotAndTransPair(relativeMarker->pose());
            relativeMarker2camNow.rotation = rotAndTrans.first;
            relativeMarker2camNow.translation = rotAndTrans.second;
            relativeMarker2camNow.averageLinkConfidence = relativeMarker->confidence();
            relativeMarker2camNow.maxDistanceToEntry = 0.;

            relativeMarker->visible = !(relativeMarker2camNow.averageLinkConfidence < thConfVisible);

            StampedTransformationWithConfidence world2relativeMarkerNow = multiplySTWC(invertSTCW(relativeMarker2camNow), world2camNow);
            relativeMarker->relativePose = rotAndTransPair2Matrix(rotAndTransPair{world2relativeMarkerNow.rotation, world2relativeMarkerNow.translation});

            emit relativeMarker->relativePoseUpdated();
            emit relativeMarker->visibilityUpdated();
        }
    }

    // Update the position of the world center marker.
    worldCenterMarker->relativePose = rotAndTransPair2Matrix(rotAndTransPair{world2camNow.rotation, world2camNow.translation});
    emit worldCenterMarker->relativePoseUpdated();

    #ifdef USE_MONITORING_FUNCTIONALITY
    emit transformationUpdate("world2cam", tsNow, worldCenterMarker->relativePose, world2camNow.averageLinkConfidence, world2camNow.maxDistanceToEntry);
    #endif
}

// Monitoring functions for the qml interface
#ifdef USE_MONITORING_FUNCTIONALITY
void MarkerModel::startMonitoring(){

    for(Landmark* l : relativeMarkers){
         emit registerLinkUpdateToMonitor(l->identifier.toStdString(), camID);
    }

    emit startMonitor();
}

void MarkerModel::stopMonitoring(){
    emit stopMonitor();
}
#endif

// Set the world center marker and establish connection to get position updates
void MarkerModel::setWorldCenterMarker( Landmark* worldCenterMarker) {

    if(worldCenterMarker == nullptr)
        return;

    this->worldCenterMarker = worldCenterMarker;
    this->worldID = (worldCenterMarker->identifier).toStdString();

    connect(worldCenterMarker, &Landmark::changed, this, &MarkerModel::markerPositionUpdated);
}

Landmark* MarkerModel::getWorldCenterMarker() { return worldCenterMarker; }

QQmlListProperty<Landmark> MarkerModel::worldCenterRelativeMarkers(){
    return QQmlListProperty<Landmark>(this, this,
                                      &MarkerModel::appendWorldCenterRelativeMarker,
                                      &MarkerModel::worldCenterRelativeMarkersCount,
                                      &MarkerModel::worldCenterRelativeMarker,
                                      &MarkerModel::clearWorldCenterRelativeMarkers);
}

void MarkerModel::appendWorldCenterRelativeMarker(Landmark* worldCenterRelativeMarker) {

    connect(worldCenterRelativeMarker, &Landmark::changed, this, &MarkerModel::markerPositionUpdated);

    relativeMarkers.append(worldCenterRelativeMarker);
}

int MarkerModel::worldCenterRelativeMarkersCount() const {
    return relativeMarkers.count();
}

Landmark* MarkerModel::worldCenterRelativeMarker(int i) const {
    return relativeMarkers.at(i);
}

void MarkerModel::clearWorldCenterRelativeMarkers() {
    return relativeMarkers.clear();
}

void MarkerModel::appendWorldCenterRelativeMarker(QQmlListProperty<Landmark>* list, Landmark* p) {
    reinterpret_cast< MarkerModel* >(list->data)->appendWorldCenterRelativeMarker(p);
}

void MarkerModel::clearWorldCenterRelativeMarkers(QQmlListProperty<Landmark>* list) {
    reinterpret_cast< MarkerModel* >(list->data)->clearWorldCenterRelativeMarkers();
}

Landmark* MarkerModel::worldCenterRelativeMarker(QQmlListProperty<Landmark>* list, int i) {
    return reinterpret_cast< MarkerModel* >(list->data)->worldCenterRelativeMarker(i);
}

int MarkerModel::worldCenterRelativeMarkersCount(QQmlListProperty<Landmark>* list) {
    return reinterpret_cast< MarkerModel* >(list->data)->worldCenterRelativeMarkersCount();
}

// Functions for monitoring
#ifdef USE_MONITORING_FUNCTIONALITY
void MarkerModelMonitor::registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame){

    std::string linkID = srcFrame+destFrame;

    // Check if the link is already monitored.
    if(monitoredLinkUpdates.find(linkID) != monitoredLinkUpdates.end())
        return;                 // Link already registred to be monitored

    // Create container needed for tracking.
    monitoredLinkUpdates.insert({linkID, std::list<LinkUpdate>()});
    monitoredLinkIdentifier.insert({linkID, {srcFrame, destFrame}});
}

void MarkerModelMonitor::monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
                                           const rotAndTransPair &transf, const float &conf) {

    if(!currentlyMonitoring)
        return;

    std::string linkID = srcFrame+destFrame;

    // Check if link is monitored.
    auto iter2monitoredLinkUpdates = monitoredLinkUpdates.find(linkID);
    if(iter2monitoredLinkUpdates == monitoredLinkUpdates.end())
        return;                 // Link is not monitored.

    // Add entry raw to the corresponding container.
    std::list<LinkUpdate> &refToUpdates = ((*iter2monitoredLinkUpdates).second);
    if(refToUpdates.size() < MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK)
        refToUpdates.push_back(LinkUpdate{ts, transf, conf});
}

void MarkerModelMonitor::registerTransformationToMonitor(const std::string &transID){

    if(monitoredTransformation.find(transID) != monitoredTransformation.end())
        return;                 // Transformation is already registered to be monitored.

    monitoredTransformation.insert({transID, std::list<TransformationUpdate>()});
}

void MarkerModelMonitor::monitorTransformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                                                     const float &averageLinkConfidence, const float &maxDistanceToEntry){
    if(!currentlyMonitoring)
        return;

    auto iter2monitoredTransformations = monitoredTransformation.find(transID);
    if(iter2monitoredTransformations == monitoredTransformation.end())
        return;             // Transformation is not monitored.

    // Add entry to the corresponding container.
    std::list<TransformationUpdate> &refToTransf = ((*iter2monitoredTransformations).second);
    if(refToTransf.size() < MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION)
        refToTransf.push_back(TransformationUpdate{ts, matrix2rotAndTransPair(trans), averageLinkConfidence, maxDistanceToEntry});
}

void MarkerModelMonitor::startMonitoring() {

    if(currentlyMonitoring)
        return;         // Already monitoring.

    monitoringStartedAt = std::chrono::high_resolution_clock::now();

    currentlyMonitoring = true;
}

void MarkerModelMonitor::stopMonitoring() {

    if(!currentlyMonitoring)
        return;         // Cannot stop monitoring if it has never started.

    // Create folder where we store all the dumped files for each analysis.
    QDateTime currentTime = QDateTime::currentDateTime();
    QString folderPath = PATH + "Recording_" + currentTime.toString("ddMMyy_HHmmss") + "/";

    QDir dir;
    int dirExists = dir.exists(folderPath);
    if( !dirExists )
        dir.mkdir(folderPath);

    /* REMARK:
     * At the moment all possible analyses are done and written to a file for all tracked link
     * and transformation updates as soon the monitoring is stopped. It would make sense to make the analyses
     * choosable through a gui.ation strongly depends in a later step. */

    writeAllLinkUpateRecordsToFile(folderPath);
    writeAllTransformationUpateRecordsToFile(folderPath);

    // Clean up all container for the next monitoring session.
    monitoredLinkIdentifier.clear();
    monitoredLinkUpdates.clear();
    monitoredTransformation.clear();

    currentlyMonitoring = false;
}

void MarkerModelMonitor::writeAllTransformationUpateRecordsToFile(const QString &path){

    // Do a transformation update analysis for all monitored transformations and write
    // each of it to a seperate file.
    auto iter2 = monitoredTransformation.begin();
    while(iter2 != monitoredTransformation.end()){

        QString transID = QString::fromStdString((*iter2).first);

        std::list<TransformationUpdate> recordToWrite = (*iter2).second;

        for(TransformationUpdate& tu : recordToWrite){
            tu.timeSinceFirstRecordedUpdate = ((std::chrono::duration_cast<std::chrono::milliseconds>(tu.time - monitoringStartedAt)).count());
            tu.transformationID;
        }

        writeSingleTransforamtionUpdateRecordToFile(recordToWrite, path + "Transformation_Update_Record_" + transID);
        iter2++;
    }
}

void MarkerModelMonitor::writeAllLinkUpateRecordsToFile(const QString &path){

    // Do a link update analysis for all monitored link updates and write each of it to a seperate file.
    auto iter = monitoredLinkUpdates.begin();
    while(iter != monitoredLinkUpdates.end()){

        std::string linkID = (*iter).first;
        QString srcFrame = QString::fromStdString((monitoredLinkIdentifier.at(linkID)).first);
        QString dstFrame = QString::fromStdString((monitoredLinkIdentifier.at(linkID)).second);

        std::list<LinkUpdate> recordToWrite = monitoredLinkUpdates.at(linkID);

        for(LinkUpdate& lu: recordToWrite){
            lu.timeSinceFirstRecordedUpdate = ((std::chrono::duration_cast<std::chrono::milliseconds>(lu.time - monitoringStartedAt)).count());
            lu.srcFrame = srcFrame;
            lu.dstFrame = dstFrame;
        }

        writeSingleLinkUpdateRecordToFile(recordToWrite, path  + "Link_Update_Record_" + srcFrame + "_" + dstFrame, false);
        iter++;
    }
}

void MarkerModelMonitor::writeSingleLinkUpdateRecordToFile(std::list<LinkUpdate> &output, const QString &path, bool appendSRCandDST){

    const QString lineSeperator = ",", newLine = "\n";

    QFile file(path + ".m");
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << file.errorString();
        return;
    }

    QTextStream out(&file);

    for(LinkUpdate lu : output){

        out << QString::number(lu.timeSinceFirstRecordedUpdate)     << lineSeperator
            << QString::number(lu.transformation.first.scalar())    << lineSeperator
            << QString::number(lu.transformation.first.x())         << lineSeperator
            << QString::number(lu.transformation.first.y())         << lineSeperator
            << QString::number(lu.transformation.first.z())         << lineSeperator
            << QString::number(lu.transformation.second.x())        << lineSeperator
            << QString::number((lu.transformation.second.y()))      << lineSeperator
            << QString::number(lu.transformation.second.z())        << lineSeperator
            << QString::number(lu.confidence);

        if(appendSRCandDST){
            out << lineSeperator
                << lu.srcFrame                                          << lineSeperator
                << lu.dstFrame;
        }
        out << newLine;

    }

    file.close();
    if(file.error()){
        qDebug() << file.errorString();
        return;
    }
}

void MarkerModelMonitor::writeSingleTransforamtionUpdateRecordToFile(std::list<TransformationUpdate> &output, const QString &path){

    const QString lineSeperator = ",", newLine = "\n";

    QFile file(path + ".m");
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << file.errorString();
        return;
    }

    QTextStream out(&file);

    for(TransformationUpdate tu : output){

        out << QString::number(tu.timeSinceFirstRecordedUpdate)     << lineSeperator
            << QString::number(tu.transformation.first.scalar())    << lineSeperator
            << QString::number(tu.transformation.first.x())         << lineSeperator
            << QString::number(tu.transformation.first.y())         << lineSeperator
            << QString::number(tu.transformation.first.z())         << lineSeperator
            << QString::number(tu.transformation.second.x())        << lineSeperator
            << QString::number((tu.transformation.second.y()))      << lineSeperator
            << QString::number(tu.transformation.second.z())        << newLine;

    }

    file.close();
    if(file.error()){
        qDebug() << file.errorString();
        return;
    }
}
#endif
