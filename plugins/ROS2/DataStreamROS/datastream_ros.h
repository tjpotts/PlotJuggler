#ifndef DATASTREAM_ROS2_TOPIC_H
#define DATASTREAM_ROS2_TOPIC_H

#include <QtPlugin>

#include "PlotJuggler/datastreamer_base.h"

#include "../dialog_select_ros_topics.h"

#include "RosManager.h"

class DataStreamROS : public DataStreamer
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.ROS2DataStreamer" "../datastreamer.json")
    Q_INTERFACES(DataStreamer)

public:

    DataStreamROS();

    virtual bool start(QStringList* selected_datasources) override;

    virtual void shutdown() override;

    virtual bool isRunning() const override;

    virtual ~DataStreamROS() override;

    virtual const char* name() const override { return "ROS2 Topic Subscriber"; }

    virtual bool xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const override;

    virtual bool xmlLoadState(const QDomElement &parent_element) override;

    virtual void addActionsToParentMenu(QMenu* menu) override;

    virtual std::vector<QString> appendData(PlotDataMapRef& destination) override
    {
        _destination_data = &destination;
        return DataStreamer::appendData(destination);
    }

public slots:
    void message_received(QString topic);

signals:
    void topicListDialogOpening();
    void topicListDialogClosed();
    void clearSubscriptionsRequest();
    void subscriptionRequest(QString topic);

private:

    RosManager _ros_manager;

    PlotDataMapRef* _destination_data;

    bool _running;

    DialogSelectRosTopics::Configuration _topic_config;

};

#endif

