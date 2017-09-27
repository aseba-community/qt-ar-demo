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

/*! \typedef rotAndTransPair
 * \brief Represents a transformation consisting of a quaternion representing
 * the rotation and a vector representing the translation.
 */
typedef std::pair<QQuaternion, QVector3D> rotAndTransPair;

/*! \def USE_MONITORING_FUNCTIONALITY
 * \brief If defined the monitoring functionality of the marker model is compiled and
 * can be used. It allows to write transformation updates received from the tracker as well
 * as calculated output transformations to files.
 */
#define USE_MONITORING_FUNCTIONALITY

Q_DECLARE_METATYPE(Timestamp)
Q_DECLARE_METATYPE(rotAndTransPair)
Q_DECLARE_METATYPE(std::string)

#ifdef USE_MONITORING_FUNCTIONALITY
class MarkerModelMonitor;
#endif

/*! \class MarkerModel
 * \brief Creates a new relationship between multiple markers. For every marker added to the model
 * the marker model allows to calculate the position of this marker relativ to the special world
 * center marker. The behavior of the marker model is determined through different parameter which
 * for example specify whether a marker is visible or not.
 */
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

    /*! Part of the qt-QML interface to set the world center marker. */
    Q_PROPERTY(Landmark* worldCenterMarker READ getWorldCenterMarker WRITE setWorldCenterMarker)
    /*! Part of the qt-QML interface to set the relative markers. */
    Q_PROPERTY(QQmlListProperty<Landmark> worldCenterRelativeMarkers READ worldCenterRelativeMarkers)

    /*! Part of the qt-QML interface to set the visibility threshold thConfVisible. */
    Q_PROPERTY(double thConfVisible READ getThConfVisible WRITE setThConfVisible)
    /*! Part of the qt-QML interface to set the update threshold thConfUpdate. */
    Q_PROPERTY(double thConfUpdate READ getThConfUpdate WRITE setThConfUpdate)
    /*! Part of the qt-QML interface to set the threshold thLastUpdate. */
    Q_PROPERTY(double thLastUpdate READ getThLastUpdate WRITE setThLastUpdate)
    /*! Part of the qt-QML interface to set the threshold thFixUpdate. */
    Q_PROPERTY(double thFixUpdate READ getThFixUpdate WRITE setThFixUpdate)

    /*! Part of the qt-QML interface to set the useTransMem. */
    Q_PROPERTY(bool useTransMem READ getUseTransMem WRITE setUseTransMem)

    #ifdef USE_MONITORING_FUNCTIONALITY
    // QML Interface for the monitoring
    Q_INVOKABLE void startMonitoring();
    Q_INVOKABLE void stopMonitoring();
    #endif

    /*! \fn void updateModel()
     * \brief Triggers the calculation of the output, i.e. the calculation of the
     * transformation mapping from the world center marker to each markers as well
     * as the calculation mapping from the world center marker to the camera.
     */
    Q_INVOKABLE void updateModel();

    /*! Interface for qt-QML. */
    QQmlListProperty<Landmark> worldCenterRelativeMarkers();
    void appendWorldCenterRelativeMarker(Landmark* worldCenterRelativeMarker);
    int worldCenterRelativeMarkersCount() const;
    Landmark* worldCenterRelativeMarker(int i) const;
    void clearWorldCenterRelativeMarkers();

    /*! \var double thConfVisible
     * \brief Threshold for the visiblity of a marker. If the confidence
     * of a marker is below this threshold then the marker is set to be
     * not visible. Value can be set through the qt-QML interface.
     */
    double thConfVisible{0.3};
    double getThConfVisible(){ return thConfVisible; }     void setThConfVisible(double th){ thConfVisible = th; }

    /*! \var double thConfUpdate
     * \brief Threshold for the update. If the current confidence of a marker
     * is above this threshold then the transformation is stored in TransMem.
     * Value can be set through the qt-QML interface.
     */
    double thConfUpdate{0.45};
    double getThConfUpdate(){ return thConfUpdate; }       void setThConfUpdate(double th){ thConfUpdate = th; }

    /*! \var double thLastUpdate
     * \brief If TransMem was updated within the time specified by this
     * threshold then the marker is visible.
     * Value can be set through the qt-QML interface.
     */
    double thLastUpdate{250};    double getThLastUpdate(){ return thLastUpdate; }       void setThLastUpdate(double th){ thLastUpdate = th; }

   /*! \var double thFixUpdate
     * \brief If two markers are seen together within the time specified through this
     * threshold then the fix transformation between these two markers is updated.
     * Value can be set through the qt-QML interface.
     */
    double thFixUpdate{250};     double getThFixUpdate(){ return thFixUpdate; }         void setThFixUpdate(double th){ thFixUpdate = th; }

    /*! \var bool useTransMem
     * \brief Determines whether should be TransMem used or not.
     * This functionality is implemented to demonstrate the functionality
     * of TransMem.
     */
    bool useTransMem{true};      bool getUseTransMem(){ return useTransMem; }           void setUseTransMem(bool b) { useTransMem = b; }

public slots:

    /*! \fn void markerPositionUpdated()
     * \brief Invoked whenever the pose of a marker is updated through the tracker.
     */
    void markerPositionUpdated();

private:

    /*! \var Landmark* worldCenterMarker
     * \brief The marker model calculates the mapping between this special world center marker
     * and all relative markers every time void updateModel() is triggered. Additionally the position
     * of this marker relative to the camera is updated.
     */
    Landmark* worldCenterMarker;

    /*! \var QVector<Landmark*> relativeMarkers
     * \brief Stores all the relative marker whose position relative to the world center marker
     * is calculated whenever void updateModel() is triggered.
     */
    QVector<Landmark*> relativeMarkers;

    /*! \fn StampedTransformationWithConfidence multiplySTWC( const StampedTransformationWithConfidence &lhs, const StampedTransformationWithConfidence &rhs)
     * \brief Multiplies the transformations encoded in the two arguments \a lhs and \a rhs. If \a lhs encodes a mapping from B to C and \a rhs encodes a mapping
     * from A to B then the returned StampedTransformationWithConfidence object encodes a mapping from A to C. The time field of the returned object is set to the
     * time field of \a lhs. For the confidence and the max distance to entry fields the larger value of the corresponding field from either \a lhs or \a rhs is chosen.
     * \return StampedTransformationWithConfidence object which encodes the multiplied transformation.
     */
    StampedTransformationWithConfidence multiplySTWC( const StampedTransformationWithConfidence &lhs, const StampedTransformationWithConfidence &rhs);

    /*! \fn StampedTransformationWithConfidence averageSTWCifEqual(StampedTransformationWithConfidence &lhs, StampedTransformationWithConfidence &rhs)
     * \brief Compares the transformations encoded through \a lhs and \a rhs. The comparison is done with
     * QVector4D compareRotAndTransPair(). The two StampedTransformationWithConfidence objects are averaged
     * if this is the case. The averaging is done component-wise (\see QQuaternion avgAndNormalizeQuaternions(..)
     * and QVector3D avgVector3D(..)). The time and maxDistanceToEntry for the result is chosen from \a lhs.
     * The confidence of the result is set to the average of the confidence value of \a lhs and \a rhs.
     * If the two transformations are not equal then the \a lhs is returned as a result.
     */
    StampedTransformationWithConfidence averageSTWCifEqual(StampedTransformationWithConfidence &lhs, StampedTransformationWithConfidence &rhs);

    /*! \fn  StampedTransformationWithConfidence invertSTCW(StampedTransformationWithConfidence &lhs)
     * \brief Inverts the transformation submitted in the argument. The other fields: time, confidence and max distance to
     * entry remain untouched.
     * \param lhs StampedTransforamtionWithConfidence which encode the transformation which is inverted by the function.
     * \return StampedTransformationWithConfidence object which encodes inverted tranformation.
     */
    StampedTransformationWithConfidence invertSTCW(StampedTransformationWithConfidence &lhs);

    /*! \var const std::string camID
     * \brief Identifier of the camera frame. (Coordinate system)
     */
    const std::string camID {"cam"};

    /*! \var std::string worldID
     * \brief Identifier of the world center marker. Set through
     * void setWorldCenterMarker( Landmark* worldCenterMarker).
     */
    std::string worldID;

    /*! \fn void setWorldCenterMarker(Landmark* worldCenterMarker)
     * \brief Sets the world center marker and its identifier worldID.
     * \param worldCenterMarker Pointer to the world center marker.
     */
    void setWorldCenterMarker( Landmark* worldCenterMarker);

    /*! \fn Landmark* getWorldCenterMarker
     * \brief Returns a pointer to the world center marker
     * \return Pointer to the world center marker. Returns null pointer if the world center
     * marker is not set.
     */
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
    /*! \fn void linkUpdate(const std::string &srcFrame, const std::string &dstFrame, const Timestamp &ts,
     *                      const rotAndTransPair &transf, const float &conf)
     * \brief This signal is emited every time the position of a marker is updated through the function
     * updateLinkNow(..). It is used to inform the monitor about the update.
     * \param srcFrame Identifier of the source coordinate system.
     * \param dstFrame Identifier of the destination coordinate system.
     * \param ts Timestamp when the update occured.
     * \param transf Transformation mapping from srcFrame to dstFrame.
     * \param conf Confidence value as an indication how good the updated transformation is.
     */
    void linkUpdate(const std::string &srcFrame, const std::string &dstFrame, const Timestamp &ts,
                    const rotAndTransPair &transf, const float &conf);

    // Object can emit this signal whenever a calculated transformation (result, output to QML model) was updated.
    void transformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                              const float &avgLinkConfidence, const float &avgDistanceToEntry);


    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);
    void registerTransformationToMonitor(const std::string &transID);

    /*! \fn startMonitor()
     * \brief Starts the monitoring. All registered links (LinkUpdate) and transformations are stored
     * until void stopMonitor() is called or the maximal number of entries (MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK,
     * MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION) is reached.
     */
    void startMonitor();

    /*! \fn stopMonitor()
     * \brief Stops the monitoring. For each registered link or transformation a seperate file is generated
     * containing the corresponding transformations as well as additional information such as confidence or
     * timestamp.
     */
    void stopMonitor();
#endif
};

#ifdef USE_MONITORING_FUNCTIONALITY
/*! \struct LinkUpdate
 * \brief Stores a single update of a link direct from the tracker.
 */
struct LinkUpdate {
    Timestamp time;
    rotAndTransPair transformation;
    double confidence;

    double timeSinceFirstRecordedUpdate;
    QString srcFrame;
    QString dstFrame;
};

/*! \struct TransformationUpdate
 * \brief Stores as single transformation calculated as output of
 * the marker model.
 */
struct TransformationUpdate  {
    Timestamp time;
    rotAndTransPair transformation;
    float avgerageLinkConfidence;
    float maxDistanceToEntry;

    double timeSinceFirstRecordedUpdate;
    QString transformationID;
};
#endif

#ifdef USE_MONITORING_FUNCTIONALITY
/*! \class MarkerModelMonitor
 * \brief Implements the monitoring functionality to monitor link updates
 * as well as the transformations calculated as output of the marker model.
 */
class MarkerModelMonitor : public QObject{

    Q_OBJECT

public slots:

    /*! \fn void monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
     *                             const rotAndTransPair &transf, const float &conf)
     * \brief Slot triggered whenever the signal MarkerModel::linkUpdate(const std::string &srcFrame, const std::string
     * &dstFrame, const Timestamp &ts, const rotAndTransPair &transf, const float &conf) is emitted. For more information
     * see documentation of this signal.
     */
    void monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
                           const rotAndTransPair &transf, const float &conf);

    /*! \fn  void monitorTransformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
     *                               const float &avgLinkConfidence, const float &maxDistanceToEntry)
     * \brief Slot triggered whenever the signal MarkerModel::transformationUpdate(const std::string &transID, const Timestamp
     *  &ts, const QMatrix4x4 &trans, const float &avgLinkConfidence, const float &avgDistanceToEntry) is emitted. For more
     * information see documentation of this signal.
     */
    void monitorTransformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                                     const float &avgLinkConfidence, const float &maxDistanceToEntry);

    /*! \fn void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame)
     * \brief After registration the monitor records the link updates from \a srcFrame to \a destFrame
     * as soon as the monitoring is started through startMonitoring().
     */
    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);

    /*! \fn void registerTransformationToMonitor(const std::string &transID)
     * \brief After registration the monitor records the transformation update with the
     * identifier \transID as soon as the monitoring is started through startMonitoring().
     */
    void registerTransformationToMonitor(const std::string &transID);

    /*! \fn void startMonitoring()
     * Starts the monitoring. For more information \see MarkerModel::startMonitor().
     */
    void startMonitoring();

    /*! \fn void stopMonitoring()
     * Stops the monitoring. For more information \see MarkerModel::stopMonitor().
     */
    void stopMonitoring();

protected:

    /*! \var std::unordered_map<std::string, std::list<LinkUpdate> > monitoredLinkUpdates
     * \brief Container which sotres the monitred link updates.
     */
    std::unordered_map<std::string, std::list<LinkUpdate> > monitoredLinkUpdates;

    /*! \var std::unordered_map<std::string, std::pair<std::string, std::string> > monitoredLinkIdentifier
     * \brief Stores for each monitored link the identifier of the source and the destination.
     */
    std::unordered_map<std::string, std::pair<std::string, std::string> > monitoredLinkIdentifier;

    /*! \var std::unordered_map<std::string, std::list<TransformationUpdate> > monitoredTransformation
     * \brief Container which stores the monitored transformations.
     */
    std::unordered_map<std::string, std::list<TransformationUpdate> > monitoredTransformation;

    /*! \var const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK
     * \brief Maximal number of link updates the monitor can store. No more link updates
     * are recorded if reached.
     */
    const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK = 1000000;

    /*! \var const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION
     * \brief Maximal number of transformation updates the monitor can store. No more
     * transformation updates are stored if reached.
     */
    const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION = 1000000;

    /*! \var const QString PATH
     * \brief Location where the files containing the recorded transformations are dumped
     * when the monitoring is stopped.
     */
    const QString PATH = "..";

    /*! \var Timestamp monitoringStartedAt
     * \brief Time when the monitoring was triggered through startMonitoring().
     */
    Timestamp monitoringStartedAt;

    /*! \var bool currentlyMonitoring
     * \brief True if the monitor is currently monitoring, false otherwise.
     */
    bool currentlyMonitoring = false;

    /*! \fn void writeAllTransformationUpateRecordsToFile(const QString &path)
     * \brief Creates for each registered transformation a file which contains all recorded updates.
     * The file is dumped at the location specified by \a path.
     */
    void writeAllTransformationUpateRecordsToFile(const QString &path);

    /*! \fn void writeAllLinkUpateRecordsToFile(const QString &path)
     * \brief Creates for each registered link a file which contains all recorded link updates.
     * The file is dumped at the location specified by \a path.
     */
    void writeAllLinkUpateRecordsToFile(const QString &path);

    /*! \fn void writeSingleLinkUpdateRecordToFile(std::list<LinkUpdate>& output, const QString &path, bool appenSRCandDST)
     * \brief Writes all LinkUpdates stored in \a output into a file which is dumped at the location
     * specified by \a path. If \a appendSRCandDST is true, then the identifier of the source and the destination connected through
     * the monitored link is appended to each record.
     */
    void writeSingleLinkUpdateRecordToFile(std::list<LinkUpdate>& output, const QString &path, bool appendSRCandDST);

    /*! \fn void writeSingleTransforamtionUpdateRecordToFile(std::list<TransformationUpdate>& output, const QString &path)
     * \brief Writes all TransformationUpdates stored in \a output into a file which is dumped at the location
     * specified by \a path.
     */
    void writeSingleTransforamtionUpdateRecordToFile(std::list<TransformationUpdate>& output, const QString &path);
};
#endif

#endif // MARKERMODEL_H
