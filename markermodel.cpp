#include "markermodel.h"

/********************
 * HELPER FUNCTIONS *
 ********************/

/* REMARK:
 * Is it bad practice to define functions used by multiple classes and which not belong
 * to the class in a strong sense to one class outside without any class. */

QMatrix4x4 qPair2Matrix(const qPair &qp){

    QMatrix4x4 ret(qp.first.toRotationMatrix());
    ret(0,3) = qp.second.x();   ret(1,3) = qp.second.y(); ret(2,3) = qp.second.z();
    return ret;

}

qPair matrix2qPair(const QMatrix4x4 &m){

    float data[]{ m(0,0),m(0,1),m(0,2),
                  m(1,0),m(1,1),m(1,2),
                  m(2,0),m(2,1),m(2,2)
                };

    QMatrix3x3 rM(data);

    return qPair{QQuaternion::fromRotationMatrix(rM), QQuaternion(0, m(0,3), m(1,3), m(2,3))};
}

QVector4D compareqPair(const qPair &qp1, const qPair &qp2){

    const double toRad = M_PI/180;

    // We first split each transformation in a translation vector vTra,
    // a rotation axis vRotA and an angle fAngl.
    QVector3D vTra1 = QVector3D(qp1.second.x(), qp1.second.y(), qp1.second.z()),
              vTra2 = QVector3D(qp2.second.x(), qp2.second.y(), qp2.second.z());

    QVector3D vRotA1, vRotA2; float fAngl1, fAngl2;

    qp1.first.getAxisAndAngle(&vRotA1, &fAngl1);
    qp2.first.getAxisAndAngle(&vRotA2, &fAngl2);

    // Make sure the two rotation axes are comparable.
    if(vRotA1.z() < 0 ){ vRotA1 *= -1; fAngl1 = 360 - fAngl1; }
    if(vRotA2.z() < 0 ){ vRotA2 *= -1; fAngl2 = 360 - fAngl2; }

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

    // Calculate the distance between a point perpendicular to the rotation axis
    // with length one and this point rotated by the angle difference.
    double distanceNormalPointR = sqrt(2*(1-cos((fAngl1-fAngl2)*toRad)));

    // Calculate the distance between the two rotation vectors.
    vRotA1.normalize(); vRotA2.normalize();
    double distanceNormalR = fabs((vRotA1 - vRotA2).length());

    // Return a vector encoding the "difference" between two transformations.
    // For equal transformation the method returns QVector4D(0,0,0,0).
                        //x             //y                 //z                     //w
    return QVector4D(   ratioLenT,     distanceNormalT,    distanceNormalPointR,   distanceNormalR);
}

QQuaternion avgAndNormalizeQuaternions(const QQuaternion &q1, const QQuaternion &q2){

    QQuaternion ret = QQuaternion( (q1.scalar() + q2.scalar()) / 2.,
                                   (q1.x() + q2.x()) / 2.,
                                   (q1.y() + q2.y()) / 2.,
                                   (q1.z() + q2.z()) / 2. );

    return ret.normalized();
}

QQuaternion avgQuaternions(const QQuaternion &q1, const QQuaternion &q2){

    QQuaternion ret = QQuaternion( (q1.scalar() + q2.scalar()) / 2.,
                                   (q1.x() + q2.x()) / 2.,
                                   (q1.y() + q2.y()) / 2.,
                                   (q1.z() + q2.z()) / 2.
                                 );
    return ret;
}

bool equalTransformation(const qPair &qp1, const qPair &qp2){

    // Parameter to decide if to transforamtion are equal.              ratioLenT   distanceNormalT  distanceNormalPointR  distanceNormalR
    const QVector4D thTransformationEquality = QVector4D(        0.2,        0.2,             0.2,                  0.2);

    QVector4D diff = compareqPair(qp1, qp2);

    return diff.x() < thTransformationEquality.x() && diff.y() < thTransformationEquality.y() &&
           diff.z() < thTransformationEquality.z() && diff.w() < thTransformationEquality.w();
}

/***************
 * MARKERMODEL *
 ***************/

// Constructor

MarkerModel::MarkerModel(){

    // Necessary to allow connections to marker model monitor
    qRegisterMetaType<Timestamp>(); qRegisterMetaType<qPair>(); qRegisterMetaType<std::string>();

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

// Main functions for the qml interface

void MarkerModel::updateLinkNow(const QString &srcFrame, const QString &destFrame, const QMatrix4x4 &trans, float conf){

    Timestamp tsNow = std::chrono::high_resolution_clock::now();
    qPair qp = matrix2qPair(trans);
    std::string srcF = srcFrame.toStdString(); std::string destF = destFrame.toStdString();

    // Signal to monitor, let it now there is a new update.
    emit linkUpdate(srcF, destF, tsNow, qp, conf);

    // Model specific implementation

    // If confidence is high enough, that is greater than a threshold,
    // we update transmem with the link.
    if(conf >= thConfidenceMarkerUpdate)
        registerLink(srcF, destF, tsNow, qp.first, qp.second, conf);
    else
        try { updateLinkConfidence(srcF, destF, conf); }
        catch(NoSuchLinkFoundException e) { /* Quality can just be updated if there was already a successful update. */ }
}

void MarkerModel::updateModel(){

    Timestamp tsNow = std::chrono::high_resolution_clock::now();

    // Helper lambda's

    /* REMARK:
     * It would also be possible to implement these functions as part of the StampedAndRatedTransformation
     * object or as proteced function of the MarkerModel class. But since the implementation strongly depends
     * on implentation the model it's more consistent to implement it here. */


    // SART == StampedAndRatedTransformation
    // For more information about this type check out transmem.h.


    // Multiplication of two SART objects.
    auto SARTMultiplier = [](const StampedAndRatedTransformation &lhs, const StampedAndRatedTransformation &rhs){
        StampedAndRatedTransformation ret;

        ret.time = lhs.time;

        ret.qRot = lhs.qRot * rhs.qRot;
        ret.qTra = lhs.qRot * rhs.qTra * lhs.qRot.conjugated();
        ret.qTra = ret.qTra + lhs.qTra;

        ret.avgLinkConfidence = std::max(lhs.avgLinkConfidence, rhs.avgLinkConfidence);
        ret.maxDistanceToEntry = std::max(lhs.maxDistanceToEntry, rhs.maxDistanceToEntry);

        return ret;
    };

    // Averaging two SART objects if the encode an "equal" transformation.
    auto SARTAveragerIfEqual = [this](const StampedAndRatedTransformation &lhs, const StampedAndRatedTransformation &rhs){

        bool encodeEqualTransformation = equalTransformation(qPair{lhs.qRot, lhs.qTra}, qPair{rhs.qRot, rhs.qTra});

        if(!encodeEqualTransformation)
            return lhs;

        StampedAndRatedTransformation ret;

        ret.qRot = avgAndNormalizeQuaternions(lhs.qRot, rhs.qRot);
        ret.qTra = avgQuaternions(lhs.qTra, rhs.qTra);
        ret.avgLinkConfidence = (lhs.avgLinkConfidence + rhs.avgLinkConfidence ) / 2;
        ret.maxDistanceToEntry = (lhs.maxDistanceToEntry + rhs.maxDistanceToEntry) / 2.;
        ret.time = lhs.time;

        return ret;
    };

    // Invert the tranformation encoded in a SART object.
    auto SARTInverter = [](StampedAndRatedTransformation &lhs){

        lhs.qRot = lhs.qRot.inverted();
        lhs.qTra = -(lhs.qRot*lhs.qTra*lhs.qRot.conjugated());

        return lhs;
    };

    // ***************************************/
    // Start of model specific implementation.

    /* For the current implementation of the model the following assumptions are made:
     *
     * > The position of the marker relative to eachother is fix.
     * > The marker encoding the world center (world center marker) has to been seen once,
     *   other markers are not recognized before.
     * > If a marker is "hidden", its corresponding object is not displayed. This corresponds to
     *   the human perception of the real world - a hidden object cannot be seen. */

    /* GENERAL REMARK:
     * The three markers are hardcoded into the model at the moment. In a later stage it
     * may make sense to allow a more dynamic implementation of the model with any number
     * of tracker. */

    // Transformation needed for the model
    StampedAndRatedTransformation world2camNowWCM, orangeHouse2camNow, adaHouse2camNow,
                                  world2orangeHouseFix, world2adaHouseFix,
                                  world2adaHouseNow, world2orangeHouseNow;

    /* REMARK:
     * Handling control flow with exceptions. Is this bad practice? */

    // Fetching the transformations needed for the calculation of the model.
    try{ world2camNowWCM = getLink(worldID, camID, tsNow); }
    catch(NoSuchLinkFoundException){
        // World center marker has to be seen once, so we return here.
        return;
    }

    try{
        orangeHouse2camNow = getLink(orangeHouseID, camID, tsNow); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ adaHouse2camNow = getLink(adaHouseID, camID, tsNow); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ world2orangeHouseFix = getBestLink(worldID, orangeHouseID); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ world2adaHouseFix = getBestLink(worldID, adaHouseID); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    // Calculating the two alternative Transformation from world center marker to the camera.
    StampedAndRatedTransformation world2camNowOHM = SARTMultiplier(orangeHouse2camNow, world2orangeHouseFix),
                                  world2camNowAHM = SARTMultiplier(adaHouse2camNow, world2adaHouseFix);

    /* If the confidence for a marker is to low, that is smaller than thConfidenceMarkerVisible.,
     * we set the transformation to inactive so that the corresponding object is not displayed.
     *
     * If the confidence for a marker is greater than the threshold the marker is somehow visible.
     * If the corresponding object for the marker will be displayed is decided in a later stage. */
    world2camActiveP = !(world2camNowWCM.avgLinkConfidence < thConfidenceMarkerVisible);
    world2orangeHouseActiveP = !(orangeHouse2camNow.avgLinkConfidence < thConfidenceMarkerVisible);
    world2adaHouseActiveP = !(adaHouse2camNow.avgLinkConfidence < thConfidenceMarkerVisible);

    /* We sort the transformation from world to camera base depending on maxDistanceToEntry from smallest to greatest.
     * The smaller this value the better the link, since this link was updated most recently, and an update also means
     * the confidence for this link was over the threshold tfConfidenceMarkerUpdate. */
    std::list<StampedAndRatedTransformation> trnsWorld2camNow = {world2camNowWCM, world2camNowOHM, world2camNowAHM};
    auto cmp = [](const StampedAndRatedTransformation &lhs, const StampedAndRatedTransformation &rhs){ return lhs.maxDistanceToEntry < rhs.maxDistanceToEntry; };
    trnsWorld2camNow.sort(cmp);

    StampedAndRatedTransformation world2camBest = trnsWorld2camNow.front(); trnsWorld2camNow.pop_front();
    StampedAndRatedTransformation world2cam2Best = trnsWorld2camNow.front(); trnsWorld2camNow.pop_front();
    StampedAndRatedTransformation world2cam3Best = trnsWorld2camNow.front();

    /* If no transformation from world to camera is good enough, that is the smallest value of maxDistanceToEntry
     * for all three transformation is larger than the threshold thDistanceToLastUpdate, we set the world center
     * marker to inactive. */
    if(world2camBest.maxDistanceToEntry > thDistanceToLastUpdate)
        world2camActiveP = false; 
    /* The second best transformation has a maxDistanceToEntry value which is
     * also smaller than the threshold. If the to best transformation encode "equal"
     * transformations we average them. */
    else if(world2cam2Best.maxDistanceToEntry < thDistanceToLastUpdate &&
            world2cam3Best.maxDistanceToEntry > thDistanceToLastUpdate)
        world2camBest = SARTAveragerIfEqual(world2camBest, world2cam2Best);

    // All three tranformations from world to cam are good enough. We compare them
    // and average if there encode "equal" transformation.
    else if(world2cam3Best.maxDistanceToEntry < thDistanceToLastUpdate){
        world2camBest = SARTAveragerIfEqual(world2camBest, world2cam2Best);
        world2camBest = SARTAveragerIfEqual(world2camBest, world2cam3Best);
    }

    // We finally set the transformation for world to camera.
    world2camP = qPair2Matrix(qPair{world2camBest.qRot, world2camBest.qTra});

    /* Use the best transformation known from world to cam together with the current
     * transformation from the orange house marker to the cam to calculate this transformation
     * without having to query transmem again. */
    world2orangeHouseNow = SARTMultiplier(SARTInverter(orangeHouse2camNow), world2camBest);

    StampedAndRatedTransformation world2orangeHouseBest, world2orangeHouse2Best;

    // Find out which transformation is better, regarding the maxDistanceToEntry value
    if(world2orangeHouseNow.maxDistanceToEntry < world2orangeHouseFix.maxDistanceToEntry){
           world2orangeHouseBest = world2orangeHouseNow;
           world2orangeHouse2Best = world2orangeHouseFix;
    }
    else {
           world2orangeHouseBest = world2orangeHouseFix;
           world2orangeHouse2Best = world2orangeHouseNow;
    }

    /* If the best link from world 2 orange house is to bad, that is there exist no fix link which is good
     * enough or the the needed links were not updated a reasonable time ago, we set the orange house object
     * to inactive. */
    if(world2orangeHouseBest.maxDistanceToEntry > thDistanceToLastUpdate)
        world2orangeHouseActiveP = false;
    // If both transformation are good enough, we compare and average them if they encode the "equal" transformation.
    else if(world2orangeHouse2Best.maxDistanceToEntry < thDistanceToLastUpdate)
        world2orangeHouseBest = SARTAveragerIfEqual(world2orangeHouseBest, world2orangeHouse2Best);

    // We set the transformation for world to orange house.
    world2orangeHouseP = qPair2Matrix(qPair{world2orangeHouseBest.qRot, world2orangeHouseBest.qTra});

    // Do the same for the ada house tracker.
    world2adaHouseNow = SARTMultiplier(SARTInverter(adaHouse2camNow), world2camBest);

    StampedAndRatedTransformation world2adaHouseBest, world2adaHouse2Best;
    if(world2adaHouseNow.maxDistanceToEntry < world2adaHouseFix.maxDistanceToEntry){
        world2adaHouseBest = world2adaHouseNow;
        world2adaHouse2Best = world2adaHouseFix;
    }
    else {
        world2adaHouseBest = world2adaHouseFix;
        world2adaHouse2Best = world2adaHouseNow;
    }

    if(world2adaHouseBest.maxDistanceToEntry > thDistanceToLastUpdate)
        world2adaHouseActiveP = false;
    else if(world2adaHouse2Best.maxDistanceToEntry < thDistanceToLastUpdate)
        world2adaHouseBest = SARTAveragerIfEqual(world2adaHouseBest, world2adaHouse2Best);

    world2adaHouseP = qPair2Matrix(qPair{world2adaHouseBest.qRot, world2adaHouseBest.qTra});

    // End of model specific implementation.
    // *************************************/

    // Tell the monitor about the updated transformation.
      emit transformationUpdate(world2camID, tsNow, world2camP, world2camBest.avgLinkConfidence, world2camBest.maxDistanceToEntry);
      emit transformationUpdate(world2orangeHouseID, tsNow, world2orangeHouseP, world2orangeHouseBest.avgLinkConfidence, world2orangeHouseBest.maxDistanceToEntry);
      emit transformationUpdate(world2adaHouseID, tsNow, world2adaHouseP, world2adaHouseBest.avgLinkConfidence, world2adaHouseBest.maxDistanceToEntry);

    // Inform the qml side of the model about the updated transformation.
    emit transformationsUpdated();
}

// Monitoring functions for the qml interface

void MarkerModel::startMonitoring(){

    /* REMARK:
     * Could make the monitoring options choosable through the gui. */
    emit registerLinkUpdateToMonitor(worldID, camID);
    emit registerLinkUpdateToMonitor(orangeHouseID, camID);
    emit registerLinkUpdateToMonitor(adaHouseID, camID);

    emit registerTransformationToMonitor(world2camID);
    emit registerTransformationToMonitor(world2orangeHouseID);
    emit registerTransformationToMonitor(world2adaHouseID);

    emit startMonitor();
}

void MarkerModel::stopMonitoring(){
    emit stopMonitor();
}

// Helper functions for the qml interface

QMatrix4x4 MarkerModel::world2cam(){
    return world2camP;
}

QMatrix4x4 MarkerModel::world2orangeHouse(){
    return world2orangeHouseP;
}

QMatrix4x4 MarkerModel::world2adaHouse(){
    return world2adaHouseP;
}

bool MarkerModel::world2camActive(){
    return world2camActiveP;
}

bool MarkerModel::world2orangeHouseActive(){
    return world2orangeHouseActiveP;
}

bool MarkerModel::world2adaHouseActive(){
    return world2adaHouseActiveP;
}

/**********************
 * MARKERMODELMONITOR *
 **********************/

// Functions for monitoring

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
                                           const qPair &transf, const float &conf) {

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
                                                     const float &avgLinkConfidence, const float &maxDistanceToEntry){
    if(!currentlyMonitoring)
        return;

    auto iter2monitoredTransformations = monitoredTransformation.find(transID);
    if(iter2monitoredTransformations == monitoredTransformation.end())
        return;             // Transformation is not monitored.

    // Add entry to the corresponding container.
    std::list<TransformationUpdate> &refToTransf = ((*iter2monitoredTransformations).second);
    if(refToTransf.size() < MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION)
        refToTransf.push_back(TransformationUpdate{ts, matrix2qPair(trans), avgLinkConfidence, maxDistanceToEntry});
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
    QString folderPath = PATH + "Analysis_" + currentTime.toString("ddMMyy_HHmmss") + "/";

    QDir dir;
    int dirExists = dir.exists(folderPath);
    if( !dirExists )
        dir.mkdir(folderPath);

    /* REMARK:
     * At the moment all possible analyses are done and written to a file for all tracked link
     * and transformation updates as soon the monitoring is stopped. It would make sense to make the analyses
     * choosable through a gui.ation strongly depends in a later step. */
    doAndWriteLinkUpdateAnalyses(folderPath);

    doAndWriteTransformationUpdateAnalyses(folderPath);

    // Clean up all container for the next monitoring session.
    monitoredLinkIdentifier.clear();
    monitoredLinkUpdates.clear();
    monitoredTransformation.clear();

    currentlyMonitoring = false;
}

void MarkerModelMonitor::doAndWriteTransformationUpdateAnalyses(const QString &path){

    // Do a transformation update analysis for all monitored transformations and write
    // each of it to a seperate file.
    auto iter2 = monitoredTransformation.begin();
    while(iter2 != monitoredTransformation.end()){
        QString transID = QString::fromStdString((*iter2).first);

        TransformationUpdateAnalysis tua = TransformationUpdateAnalysis(monitoringStartedAt, transID);
        tua.doAnalysis((*iter2).second);
        writeSingleAnalysisToFile(tua, path + "Transformation_Update_Analysis_" + transID);
        iter2++;
    }
}

void MarkerModelMonitor::doAndWriteLinkUpdateAnalyses(const QString &path){

    // Do a link update analysis for all monitored link updates and write each of it to a seperate file.
    auto iter = monitoredLinkUpdates.begin();
    while(iter != monitoredLinkUpdates.end()){

        std::string linkID = (*iter).first;
        QString srcFrame = QString::fromStdString((monitoredLinkIdentifier.at(linkID)).first);
        QString destFrame = QString::fromStdString((monitoredLinkIdentifier.at(linkID)).second);

        LinkUpdateAnalysis lua = LinkUpdateAnalysis(monitoringStartedAt, srcFrame, destFrame);
        lua.doAnalysis(monitoredLinkUpdates.at(linkID));
        writeSingleAnalysisToFile(lua, path  + "Link_Update_Analysis_" + srcFrame + "_" + destFrame);
        iter++;
    }
}

void MarkerModelMonitor::writeSingleAnalysisToFile(Analysis &analysis, const QString &path){

    const QString lineSeperator = ",", newLine = "\n";

    QFile file(path + ".m");
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << file.errorString();
        return;
    }

    QTextStream out(&file);

    // If a LinkUpdateFixAnalysis is written to a file, the first data line contains
    // the fix transformation.
    if(typeid(analysis) == typeid(LinkUpdateFixAnalysis)){

        LinkUpdateFixAnalysis a = dynamic_cast<LinkUpdateFixAnalysis&>(analysis);

        out << QString::number(a.fixT.first.scalar())      << lineSeperator
            << QString::number(a.fixT.first.x())           << lineSeperator
            << QString::number(a.fixT.first.y())           << lineSeperator
            << QString::number(a.fixT.first.z())           << lineSeperator
            << QString::number(a.fixT.second.scalar())     << lineSeperator
            << QString::number(a.fixT.second.x())          << lineSeperator
            << QString::number(a.fixT.second.y())          << lineSeperator
            << QString::number(a.fixT.second.z())          << newLine;
    }

   for(AnalysisSingleResult &r : analysis.results)

       out << QString::number(r.tms)                      << lineSeperator
           << QString::number(r.tf.first.scalar())        << lineSeperator
           << QString::number(r.tf.first.x())             << lineSeperator
           << QString::number(r.tf.first.y())             << lineSeperator
           << QString::number(r.tf.first.z())             << lineSeperator
           << QString::number(r.tf.second.scalar())       << lineSeperator
           << QString::number(r.tf.second.x())            << lineSeperator
           << QString::number(r.tf.second.y())            << lineSeperator
           << QString::number(r.tf.second.z())            << lineSeperator
           << QString::number(r.ratioLenT)                << lineSeperator
           << QString::number(r.distanceNormalT)          << lineSeperator
           << QString::number(r.distanceNormalPointR)     << lineSeperator
           << QString::number(r.distanceNormalR)          << lineSeperator
           << QString::number(r.conf)                     << lineSeperator
           << QString::number(r.distanceToEntry)          << newLine;

    file.close();
    if(file.error()){
        qDebug() << file.errorString();
        return;
    }
}

/************************
 * LINK UPDATE ANALYSIS *
 ************************/

void LinkUpdateAnalysis::doAnalysis(std::list<LinkUpdate> &input){

    // We need at least two entries to do this analysis
    if(input.size() < 2)
        return;

    // Sort all updates from ealierst to latest.
    auto comperator = ([](const LinkUpdate &lu1, const LinkUpdate &lu2) { return lu1.time > lu2.time;});
    input.sort(comperator);

    auto iter = input.begin();
    LinkUpdate preLu = (*iter), curLu;
    iter++;
    while(iter != input.end()){
        curLu = (*iter);
        // Compare transformation of current update against the transformation of previous update and store result.
        QVector4D diff = compareqPair(preLu.transformation, curLu.transformation);
        results.push_front(
                    AnalysisSingleResult{
                        ((std::chrono::duration_cast<std::chrono::milliseconds>(curLu.time - tZero)).count()),
                        curLu.transformation,
                        diff.x(),
                        diff.y(),
                        diff.z(),
                        diff.w(),
                        curLu.confidence,
                        0.
              }
       );
       preLu = curLu;
       iter++;
    }
}

/****************************
 * LINK UPDATE FIX ANALYSIS *
 ****************************/

void LinkUpdateFixAnalysis::doAnalysis(std::list<LinkUpdate> &input){

    // Sort all updates from ealierst to latest.
    auto comperator = ([](const LinkUpdate &lu1, const LinkUpdate &lu2) { return lu1.time > lu2.time;});
    input.sort(comperator);

    for(LinkUpdate &l : input){
        // Compare transformation of current update against the fix transformation fixT and store result.
        QVector4D diff = compareqPair(l.transformation, fixT);
        results.push_front(
                    AnalysisSingleResult{
                        (std::chrono::duration_cast<std::chrono::milliseconds>(l.time - tZero)).count(),
                        l.transformation,
                        diff.x(),
                        diff.y(),
                        diff.z(),
                        diff.w(),
                        l.confidence,
                        0
                    }
       );
    }
}

/**********************************
 * TRANSFORMATION UPDATE ANALYSIS *
 **********************************/

void TransformationUpdateAnalysis::doAnalysis(std::list<TransformationUpdate> &input) {

    // We need at least two entries to do this analysis.
    if(input.size() < 2)
        return;

    // Sort all transformation updates from ealierst to latest.
    auto comperator = ([](const TransformationUpdate &lu1, const TransformationUpdate &lu2) { return lu1.time > lu2.time;});
    input.sort(comperator);

    auto iter = input.begin();
    TransformationUpdate preTu = (*iter), curTu;
    iter++;
    while(iter != input.end()){
        curTu = (*iter);
        // Compare transformation of current update against the transformation of the previous update and store result.
        QVector4D diff = compareqPair(preTu.transformation, curTu.transformation);
        results.push_front(
                    AnalysisSingleResult{
                        (std::chrono::duration_cast<std::chrono::milliseconds>(curTu.time - tZero)).count(),
                        curTu.transformation,
                        diff.x(),
                        diff.y(),
                        diff.z(),
                        diff.w(),
                        curTu.avgLinkConfidence,
                        curTu.maxDistanceToEntry
              }
       );
       preTu = curTu;
       iter++;
    }
}

