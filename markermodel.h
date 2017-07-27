#ifndef MARKERMODEL_H
#define MARKERMODEL_H

#include <QObject>
#include <QQmlListProperty>
#include <QThread>
#include <QDebug>
#include <QQuaternion>
#include <QVector4D>
#include <QDateTime>
#include <QFile>
#include <QDir>
#include <chrono>
#include <unordered_map>
#include <list>

#include "transmem/transmem.h"

typedef std::pair<QQuaternion, QQuaternion> qPair;

Q_DECLARE_METATYPE(Timestamp)
Q_DECLARE_METATYPE(qPair)
Q_DECLARE_METATYPE(std::string)

/************************************
 * PARAMETER FOR THE MODEL          *
 ************************************/

// A marker is considered to be visible if the confidence is greater than this threshold.
const double thConfidenceMarkerVisible = 0.4;

// Transmem is updated if the confidence of a marker is greater than this threshold.
const double thConfidenceMarkerUpdate = 0.6;

// A transformation is considered to be good enough if updated in a time smaller than this threshold.
const double thDistanceToLastUpdate = 100;       // in ms


class MarkerModelMonitor;

/*****************************
 * MARKER MODEL              *
 *****************************/

class MarkerModel : public QObject, public TransMem {

    Q_OBJECT

public:

    MarkerModel();

    // Destructor
    ~MarkerModel(){

        stopMonitoring();

        monitorThread.quit();
        monitorThread.wait();
    }

    // QML interface

    Q_PROPERTY(QMatrix4x4 world2cam READ world2cam NOTIFY transformationsUpdated)
    Q_PROPERTY(QMatrix4x4 world2orangeHouse READ world2orangeHouse NOTIFY transformationsUpdated)
    Q_PROPERTY(QMatrix4x4 world2adaHouse READ world2adaHouse NOTIFY transformationsUpdated)

    Q_PROPERTY(bool world2camActive READ world2camActive NOTIFY transformationsUpdated)
    Q_PROPERTY(bool world2adaHouseActive READ world2adaHouseActive NOTIFY transformationsUpdated)
    Q_PROPERTY(bool world2orangeHouseActive READ world2orangeHouseActive NOTIFY transformationsUpdated)

    Q_INVOKABLE void updateLinkNow(const QString& srcFrame, const QString& destFrame, const QMatrix4x4& trans, float conf);
    Q_INVOKABLE void updateModel();

    Q_INVOKABLE void startMonitoring();
    Q_INVOKABLE void stopMonitoring();

private:

    // Getter for the QML interface
    QMatrix4x4 world2cam();
    QMatrix4x4 world2orangeHouse();
    QMatrix4x4 world2adaHouse();

    bool world2camActive();
    bool world2orangeHouseActive();
    bool world2adaHouseActive();

    // Private members (P for private)
    QMatrix4x4 world2camP;
    QMatrix4x4 world2orangeHouseP;
    QMatrix4x4 world2adaHouseP;

    bool world2camActiveP;
    bool world2orangeHouseActiveP;
    bool world2adaHouseActiveP;

    // Private identifier which have to match the identifier used in qml.
    const std::string camID = "cam";
    const std::string worldID = "world";
    const std::string orangeHouseID = "orangeHouse";
    const std::string adaHouseID = "adaHouse";

    const std::string world2camID = "world2cam";
    const std::string world2orangeHouseID = "world2orangeHouse";
    const std::string world2adaHouseID = "world2adaHouse";

    // Thread which runs the monitoring.
    QThread monitorThread;

signals:

    // Signals for the monitor.

    // Object emits this signal everytime it received an update for a tranformation through
    // updateLinkNow(..) from the QML model to inform the monitor also about the update.
    void linkUpdate(const std::string &srcFrame, const std::string &dstFrame, const Timestamp &ts,
                    const qPair &transf, const float &conf);

    // Object can emit this signal whenever a calculated transformation (result, output to QML model) was updated.
    void transformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                              const float &avgLinkConfidence, const float &avgDistanceToEntry);

    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);
    void registerTransformationToMonitor(const std::string &transID);

    void startMonitor();
    void stopMonitor();

    // Signals for QML model
    void transformationsUpdated();
};

/*****************************
 * LINK UPDATE               *
 *****************************/

// Stores a single update of a link direct from the tracker.
struct LinkUpdate {
    Timestamp time;
    qPair transformation;
    double confidence;
};

/*****************************
 * TRANSFORMATION UPDATE     *
 *****************************/

// Stores a single update of a transformation used to draw an object.
struct TransformationUpdate  {
    Timestamp time;
    qPair transformation;
    float avgLinkConfidence;
    float maxDistanceToEntry;
};

/*******************************
 * ANALYSIS SINGLE RESULT      *
 ******************************/

// Stores a single result of an analysis.
struct AnalysisSingleResult {

    long tms;               // Elapsed time since analysis started in ms.

    qPair tf;                       // Transformation

    /* Values yield from the comparision of two transformation. For further
     * information check the function compareqPair(..) and the implementation of the corrensponding
     * analyses using this datatype to store their results. */
    double ratioLenT;
    double distanceNormalT;
    double distanceNormalPointR;
    double distanceNormalR;

    double conf;                    // Confidence
    double distanceToEntry;
};

/*****************************
 * TRANSFORMATION UPDATE     *
 *****************************/

// Basic type of an analysis.
struct Analysis{

    // Don't allow creation of an object of this type.
    virtual ~Analysis(){}

    Analysis(const Timestamp &tZero)
    : tZero(tZero)
    {}

    Timestamp tZero;
    // Container for all the results.
    std::list<AnalysisSingleResult> results;
};

/************************************************
 * LINK UPDATE ANALYSIS *
 ************************************************/

/* This analysis compares the for each link update, the transformation with the transformation
 * of the link update happened right before this update. The difference (ratioLenT, distanceNormalT,
 * distanceNormalPointR and distanceNormalR) for each link update is stored together with
 * additional information (time when the update happened, confidence of the update and the actual transformation)
 * in a AnalysisSingleResult instance. */

struct LinkUpdateAnalysis : Analysis {

    LinkUpdateAnalysis(const Timestamp &tsZero, const QString &srcFrame, const QString &destFrame)
    : Analysis(tsZero)
    , srcFrame(srcFrame)
    , destFrame(destFrame)
    {}

    void doAnalysis(std::list<LinkUpdate> &input);

    const QString srcFrame;
    const QString destFrame;

};

/************************************************
 * LINK UPDATE FIX ANALYSIS *
 ************************************************/

// This Analyis works like the LinkUpdateAnalysis with the exception that each link update is compared against
// a fix transformation fixT.

struct LinkUpdateFixAnalysis : LinkUpdateAnalysis {

    LinkUpdateFixAnalysis(const Timestamp &tsZero, const QString &srcFrame, const QString &destFrame, const qPair &fixT)
    : LinkUpdateAnalysis(tsZero, srcFrame, destFrame)
    , fixT(fixT)
    {}

    // Override
    void doAnalysis(std::list<LinkUpdate> &input);

    qPair fixT;
};

/************************************************
 * TRANSFORMATION UPDATE ANALYSIS               *
 ************************************************/

/* This Analyis works like the LinkUpdateAnalysis with the exception that not the confidence
 * but the average confidence (avgLinkConfidence) of a links contributing to the transformation
 * and the maxDistToEntry as a quality measurement is stored in a AnalysisSingleResult instance. */

struct TransformationUpdateAnalysis : Analysis {

    TransformationUpdateAnalysis(const Timestamp &tsZero, const QString &transID)
    : Analysis(tsZero)
    , transID(transID)
    {}

    void doAnalysis(std::list<TransformationUpdate> &input);

    const QString transID;
};

/****************************
 * MARKER MODEL              *
 ****************************/

/* REMARK:
 * Allows to monitor the link and transformation updates and to
 * dump the raw data or the already analyzed of the monitoring to files.
 *
 * Link update:    Transfromation between a source frame and a destination frame at a certain
 *                 point together with a timestamp. We get this from the tracking algorithm.
 *
 * Transformation update:   Calculated transformation from a source frame to a a destination frame
 *                          together with the quality information (avgLinkConfidence, maxDiffToEntry).
 *                          This is the output the model generates. */

class MarkerModelMonitor : public QObject{

    Q_OBJECT

public slots:

    void monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
                           const qPair &transf, const float &conf);

    void monitorTransformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                                     const float &avgLinkConfidence, const float &maxDistanceToEntry);

    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);
    void registerTransformationToMonitor(const std::string &transID);

    void startMonitoring();
    void stopMonitoring();

protected:

    // Container for the monitored link updates.
    std::unordered_map<std::string, std::list<LinkUpdate> > monitoredLinkUpdates;
    std::unordered_map<std::string, std::pair<std::string, std::string> > monitoredLinkIdentifier;

    // Container for the monitored transformation.
    std::unordered_map<std::string, std::list<TransformationUpdate> > monitoredTransformation;

    // Protect the memory.
    const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK = 1000000;
    const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION = 1000000;

    // Were do we want to store the analysis. It would make sense to make this
    // path setable via a gui in later versions.
    const QString PATH = "../thymio-ar-demo/analysis/";

    Timestamp monitoringStartedAt;

    bool currentlyMonitoring = false;

    void doAndWriteTransformationUpdateAnalyses(const QString &path);
    void doAndWriteLinkUpdateAnalyses(const QString &path);

    void writeSingleAnalysisToFile(Analysis &analysis, const QString &path);
};

#endif // MARKERMODEL_H
