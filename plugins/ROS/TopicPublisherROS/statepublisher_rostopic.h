#ifndef STATE_PUBLISHER_ROSTOPIC_H
#define STATE_PUBLISHER_ROSTOPIC_H

#include <QObject>
#include <QtPlugin>
#include <map>
#include <ros/ros.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <tf/transform_broadcaster.h>
#include "PlotJuggler/statepublisher_base.h"
#include "../shape_shifter_factory.hpp"
#include <rosbag/bag.h>

class  TopicPublisherROS: public QObject, StatePublisher
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.StatePublisher" "../statepublisher.json")
    Q_INTERFACES(StatePublisher)

public:
    TopicPublisherROS();
    virtual ~TopicPublisherROS() override;

    virtual void updateState(double current_time) override;

    virtual const char* name() const override { return "ROS Topic Re-Publisher"; }

    virtual bool enabled() const override { return _enabled; }

    void setParentMenu(QMenu *menu, QAction *action) override;

    virtual void play(double interval)  override;

public slots:
    virtual void setEnabled(bool enabled) override;

    void filterDialog(bool autoconfirm);

private:

    void broadcastTF(double current_time);

    std::map<std::string, ros::Publisher> _publishers;
    bool _enabled;
    ros::NodeHandlePtr _node;
    bool _publish_clock;
    std::unique_ptr<tf::TransformBroadcaster> _tf_publisher;
    ros::Publisher _tf_static_pub;
    ros::Publisher _clock_publisher;

    QAction* _enable_self_action;
    QAction* _select_topics_to_publish;

    std::unordered_map<std::string,bool> _topics_to_publish;

    bool toPublish(const std::string& topic_name);

    double previous_time;

    int _previous_play_index;

    void publishAnyMsg(const rosbag::MessageInstance& msg_instance);
};

#endif // DATALOAD_CSV_H
