#include "datastream_ros.h"

#include <QDebug>

DataStreamROS::DataStreamROS() :
    DataStreamer(),
    _ros_manager(),
    _destination_data(nullptr),
    _running(false),
    _topic_config()
{
    connect(this, &DataStreamROS::topicListDialogOpening,
            &_ros_manager, &RosManager::startTopicListListener);
    connect(this, &DataStreamROS::topicListDialogClosed,
            &_ros_manager, &RosManager::stopTopicListListener);
    connect(this, &DataStreamROS::clearSubscriptionsRequest,
            &_ros_manager, &RosManager::clearSubscriptions);
    connect(this, &DataStreamROS::subscriptionRequest,
            &_ros_manager, &RosManager::subscribe);
}

bool DataStreamROS::start(QStringList* selected_datasources)
{
    // Display the dialog which allows users to select ros topics to subscribe to,
    // and continuously update the list of available topics
    // We start with an empty topic list
    std::vector<std::pair<QString,QString>> dialog_topics;
    DialogSelectRosTopics dialog(dialog_topics, _topic_config);

    connect(&_ros_manager, &RosManager::topicListUpdated, this, [&dialog](QRosTopicList topic_list)
    {
        std::vector<std::pair<QString, QString>> dialog_topic_list;
        for (auto key : topic_list.keys())
        {
            dialog_topic_list.push_back(std::make_pair(key,topic_list[key]));
        }
        dialog.updateTopicList(dialog_topic_list);
    });

    emit topicListDialogOpening();
    int res = dialog.exec();
    emit topicListDialogClosed();

    _topic_config = dialog.getResult();

    // If no topics were selected, or the OK button was not pressed, do nothing
    if (res != QDialog::Accepted || _topic_config.selected_topics.empty())
    {
        return false;
    }

    // TODO: Subscribe to the selected topics
    emit clearSubscriptionsRequest();
    for (auto topic : _topic_config.selected_topics)
    {
        emit subscriptionRequest(topic);
    }

    return true;
}

bool DataStreamROS::isRunning() const { return _running; }

void DataStreamROS::shutdown()
{
    emit clearSubscriptionsRequest();

    _running = false;
}

DataStreamROS::~DataStreamROS()
{
    shutdown();
}

bool DataStreamROS::xmlSaveState(QDomDocument &doc, QDomElement &plugin_elem) const
{
    return true;
}

bool DataStreamROS::xmlLoadState(const QDomElement &parent_element)
{
    return true;
}

void DataStreamROS::addActionsToParentMenu(QMenu *menu)
{

}


