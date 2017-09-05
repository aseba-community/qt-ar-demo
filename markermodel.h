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

// Comment this out to make use of the monitoring functionality of the marker model.
// Allows to write transformation updates received from the tracker as well as transformations
// of the output to files.
//#define USE_MONITORING_FUNCTIONALITY

Q_DECLARE_METATYPE(Timestamp)
Q_DECLARE_METATYPE(rotAndTransPair)
Q_DECLARE_METATYPE(std::string)

#ifdef USE_MONITORING_FUNCTIONALITY
class MarkerModelMonitor;
#endif

class MarkerModel : public QObject, public TransMem {

    Q_OBJECT

public:

    // Destructor, just need to specify it explictly when we use the monitoring functionality.
    #ifdef USE_MONITORING_FUNCTIONALITY
    MarkerModel();

    ~MarkerModel(){
        stopMonitoring();
        monitorThread.quit();
        monitorThread.wait();
    }
    #endif

    // QML Interface for the marker model
    Q_PROPERTY(Landmark* worldCenterMarker READ getWorldCenterMarker WRITE setWorldCenterMarker)
    Q_PROPERTY(QQmlListProperty<Landmark> worldCenterRelativeMarkers READ worldCenterRelativeMarkers)

    Q_PROPERTY(double thConfVisible READ getThConfVisible WRITE setThConfVisible)
    Q_PROPERTY(double thConfUpdate READ getThConfUpdate WRITE setThConfUpdate)
    Q_PROPERTY(double thLastUpdate READ getThLastUpdate WRITE setThLastUpdate)
    Q_PROPERTY(double thFixUpdate READ getThFixUpdate WRITE setThFixUpdate)

    Q_PROPERTY(bool useTransMem READ getUseTransMem WRITE setUseTransMem)

    #ifdef USE_MONITORING_FUNCTIONALITY
    // QML Interface for the monitoring
    Q_INVOKABLE void startMonitoring();
    Q_INVOKABLE void stopMonitoring();
    #endif

    Q_INVOKABLE void updateModel();

    // Functions needed for the QML interface    
    QQmlListProperty<Landmark> worldCenterRelativeMarkers();
    void appendWorldCenterRelativeMarker(Landmark* worldCenterRelativeMarker);
    int worldCenterRelativeMarkersCount() const;
    Landmark* worldCenterRelativeMarker(int i) const;
    void clearWorldCenterRelativeMarkers();

    // Threshold for the visibility. If the current confidence of a marker is below this threshold then the marker is set to be not visible.
    double thConfVisible{0.4};   double getThConfVisible(){ return thConfVisible; }     void setThConfVisible(double th){ thConfVisible = th; }
    // Threshold for the an update. If the current confidence of a marker is above this threshold then the marker transformation is stored in TransMem.
    double thConfUpdate{0.45};   double getThConfUpdate(){ return thConfUpdate; }       void setThConfUpdate(double th){ thConfUpdate = th; }
    // If TransMem was updated within the time specified by this threshold then the marker is visible.
    double thLastUpdate{100};    double getThLastUpdate(){ return thLastUpdate; }       void setThLastUpdate(double th){ thLastUpdate = th; }
    // If two markers are seen together within the time specified through this threshold then the fix transformation between these two markers is updated.
    double thFixUpdate{250};     double getThFixUpdate(){ return thFixUpdate; }         void setThFixUpdate(double th){ thFixUpdate = th; }

    // Determines whether TransMem is used or not. Implemented mainly for presenting.
    bool useTransMem{true};      bool getUseTransMem(){ return useTransMem; }           void setUseTransMem(bool b) { useTransMem = b; }

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

    #ifdef USE_MONITORING_FUNCTIONALITY
    // Thread which runs the monitoring.
    QThread monitorThread;
    #endif

// Signals used for the monitoring.
#ifdef USE_MONITORING_FUNCTIONALITY
signals:

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
#endif
};

// Container used to store monitored transformations.
#ifdef USE_MONITORING_FUNCTIONALITY
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
#endif

// Implementation of the MarkerModelMonitor class. This class implements the monitoring.
#ifdef USE_MONITORING_FUNCTIONALITY
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

    // Were do we want to store the monitored files. It would make sense to make this
    // path setable via a gui in later versions.
    const QString PATH = "..";

    Timestamp monitoringStartedAt;

    bool currentlyMonitoring = false;

    void writeAllTransformationUpateRecordsToFile(const QString &path);
    void writeAllLinkUpateRecordsToFile(const QString &path);

    void writeSingleLinkUpdateRecordToFile(std::list<LinkUpdate>& output, const QString &path, bool appenSRCandDST);
    void writeSingleTransforamtionUpdateRecordToFile(std::list<TransformationUpdate>& output, const QString &path);
};
#endif

#endif // MARKERMODEL_H
