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

#include "thymio-ar/vision-video-filter.h"

#include "transmem/transmem.h"

typedef std::pair<QQuaternion, QVector3D> rotAndTransPair;

Q_DECLARE_METATYPE(Timestamp)
Q_DECLARE_METATYPE(rotAndTransPair)
Q_DECLARE_METATYPE(std::string)

/************************************
 * PARAMETER FOR THE MODEL          *
 ************************************/

//TODO: setable through qml interface

// A marker is considered to be visible if the confidence is greater than this threshold.
const double thConfidenceMarkerVisible = 0.45;

// Transmem is updated if the confidence of a marker is greater than this threshold.
const double thConfidenceMarkerUpdate = 0.45;

// A transformation is considered to be good enough if updated in a time smaller than this threshold.
const double thDistanceToLastUpdate = 100;       // in ms

// A fix transformation is considered to be  good all updates are within this time
const double thDistanceGoodFix = 250;

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

    // QML Interface for the marker model
    Q_PROPERTY(Landmark* worldCenterMarker READ getWorldCenterMarker WRITE setWorldCenterMarker)

    Q_PROPERTY(QQmlListProperty<Landmark> worldCenterRelativeMarkers READ worldCenterRelativeMarkers)

    Q_INVOKABLE void updateModel();

    // QML Interface for the monitoring
    Q_INVOKABLE void startMonitoring();
    Q_INVOKABLE void stopMonitoring();

    // Functions needed for the QML interface
    QQmlListProperty<Landmark> worldCenterRelativeMarkers();
    void appendWorldCenterRelativeMarker(Landmark* worldCenterRelativeMarker);
    int worldCenterRelativeMarkersCount() const;
    Landmark* worldCenterRelativeMarker(int i) const;
    void clearWorldCenterRelativeMarkers();

    int avgCounter{0};

public slots:

    // Invoked whenever the pose of a marker is updated
    void markerPositionUpdated();

private:

    // Storage for the marker
    Landmark* worldCenterMarker;
    QVector<Landmark*> relativeMarkers;

    // Model specific functions
    StampedTransformationWithConfidence multiplySTWC( const StampedTransformationWithConfidence &lhs, const StampedTransformationWithConfidence &rhs);
    StampedTransformationWithConfidence averageSTWCifEqual(StampedTransformationWithConfidence &lhs, StampedTransformationWithConfidence &rhs);
    StampedTransformationWithConfidence invertSTCW(StampedTransformationWithConfidence &lhs);

    // Identifier of the camera frame
    const std::string camID {"cam"};
    // Identifier of the world center marker, set through setWorldCenterMarker(..)
    std::string worldID;

    // Functions needed for the qml interface

    void setWorldCenterMarker( Landmark* worldCenterMarker);
    Landmark* getWorldCenterMarker();


    static void appendWorldCenterRelativeMarker(QQmlListProperty<Landmark>*, Landmark*);
    static int worldCenterRelativeMarkersCount(QQmlListProperty<Landmark>*);
    static Landmark* worldCenterRelativeMarker(QQmlListProperty<Landmark>*, int);
    static void clearWorldCenterRelativeMarkers(QQmlListProperty<Landmark>*);

    // Thread which runs the monitoring.
    QThread monitorThread;

signals:

    // Signals for the monitor.

    // Object emits this signal everytime it received an update for a tranformation through
    // updateLinkNow(..) from the QML model to inform the monitor also about the update.
    void linkUpdate(const std::string &srcFrame, const std::string &dstFrame, const Timestamp &ts,
                    const rotAndTransPair &transf, const float &conf);

    // Object can emit this signal whenever a calculated transformation (result, output to QML model) was updated.
    void transformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                              const float &avgLinkConfidence, const float &avgDistanceToEntry);

    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);
    void registerTransformationToMonitor(const std::string &transID);

    void startMonitor();
    void stopMonitor();

};

// Stores a single update of a link direct from the tracker.
struct LinkUpdate {
    Timestamp time;
    rotAndTransPair transformation;
    double confidence;

    double timeSinceFirstRecordedUpdate;
    QString srcFrame;
    QString dstFrame;
};

// Stores a single update of a transformation used to draw an object.
struct TransformationUpdate  {
    Timestamp time;
    rotAndTransPair transformation;
    float avgerageLinkConfidence;
    float maxDistanceToEntry;

    double timeSinceFirstRecordedUpdate;
    QString transformationID;
};

class MarkerModelMonitor : public QObject{

    Q_OBJECT

public slots:

    void monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
                           const rotAndTransPair &transf, const float &conf);

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

    void writeAllTransformationUpateRecordsToFile(const QString &path);
    void writeAllLinkUpateRecordsToFile(const QString &path);

    void writeSingleLinkUpdateRecordToFile(std::list<LinkUpdate>& output, const QString &path, bool appenSRCandDST);
    void writeSingleTransforamtionUpdateRecordToFile(std::list<TransformationUpdate>& output, const QString &path);
};

#endif // MARKERMODEL_H
