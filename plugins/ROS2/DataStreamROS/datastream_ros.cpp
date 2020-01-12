#include "datastream_ros.h"

#include <QDebug>
#include <QSettings>

DataStreamROS::DataStreamROS() :
    DataStreamer(),
    _ros_manager(),
    _destination_data(nullptr),
    _running(false),
    _config()
{
    connect(this, &DataStreamROS::topicListDialogOpening,
            &_ros_manager, &RosManager::startTopicListListener);
    connect(this, &DataStreamROS::topicListDialogClosed,
            &_ros_manager, &RosManager::stopTopicListListener);
    connect(this, &DataStreamROS::clearSubscriptionsRequest,
            &_ros_manager, &RosManager::clearSubscriptions);
    connect(this, &DataStreamROS::subscriptionRequest,
            &_ros_manager, &RosManager::subscribe);
    connect(&_ros_manager, &RosManager::messageReceived,
            this, &DataStreamROS::messageReceived);

    loadDefaultSettings();
}

bool DataStreamROS::start(QStringList* selected_datasources)
{
    // Display the dialog which allows users to select ros topics to subscribe to,
    // and continuously update the list of available topics
    // We start with an empty topic list
    std::vector<std::pair<QString,QString>> dialog_topics;
    DialogSelectRosTopics dialog(dialog_topics, _config);

    auto topic_list_connection = connect(&_ros_manager, &RosManager::topicListUpdated, this, [this, &dialog](QRosTopicList topic_list)
    {
        _topic_list = topic_list;
        std::vector<std::pair<QString, QString>> dialog_topic_list;
        for (auto key : topic_list.keys())
        {
            dialog_topic_list.push_back(std::make_pair(key,topic_list[key]));
        }
        dialog.updateTopicList(dialog_topic_list);
    });

    emit topicListDialogOpening();
    int res = dialog.exec();
    disconnect(topic_list_connection);
    emit topicListDialogClosed();

    _config = dialog.getResult();

    // If no topics were selected, or the OK button was not pressed, do nothing
    if (res != QDialog::Accepted || _config.selected_topics.empty())
    {
        return false;
    }

    saveDefaultSettings();

    _clock = rclcpp::Clock();
    _start_time = _clock.now().nanoseconds();

    emit clearSubscriptionsRequest();
    for (auto topic : _config.selected_topics)
    {
        emit subscriptionRequest(topic);
    }

    return true;
}

void DataStreamROS::messageReceived(QString topic, std::shared_ptr<rmw_serialized_message_t> msg)
{
    double time = (_clock.now().nanoseconds() - _start_time) * 1.0e-9;

    // TODO: Cache type info so it doesn't have to be retrieved on every message receipt
    auto type_info = ros_parser::getTypeInfo(_topic_list[topic].toStdString());
    uint8_t* deserialized_message = ros_parser::deserialize(msg, type_info);

    for (auto m : type_info.members)
    {
        auto member_name = topic + "/" + QString::fromStdString(m.path + "/" + m.name);
        auto member_value = ros_parser::getMessageMemberNumeric(deserialized_message, m);

        auto member_time = time;
        if (_config.use_header_stamp && m.header_info != nullptr)
        {
            member_time = ros_parser::getMessageTime(deserialized_message, *m.header_info.get());
        }

        auto key = member_name.toStdString();
        auto it = dataMap().numeric.find(key);
        if (it == dataMap().numeric.end())
        {
            it = dataMap().addNumeric(key);
        }
        it->second.pushBack(PlotData::Point(member_time, member_value));
    }
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
    QDomElement stamp_elem = doc.createElement("use_header_stamp");
    stamp_elem.setAttribute("value", _config.use_header_stamp ? "true" : "false");
    plugin_elem.appendChild(stamp_elem);

    // TODO: Implement discarding large arrays
    QDomElement discard_elem = doc.createElement("discard_large_arrays");
    discard_elem.setAttribute("value", _config.discard_large_arrays ? "true" : "false");
    plugin_elem.appendChild(discard_elem);

    QDomElement max_elem = doc.createElement("max_array_size");
    max_elem.setAttribute("value", _config.max_array_size ? "true" : "false");
    plugin_elem.appendChild(max_elem);

    QDomElement topics_elem = doc.createElement("selected_topics");
    for (auto topic : _config.selected_topics)
    {
        QDomElement topic_elem = doc.createElement("topic");
        topic_elem.setAttribute("name", topic);
        topics_elem.appendChild(topic_elem);
    }
    plugin_elem.appendChild(topics_elem);

    return true;
}

bool DataStreamROS::xmlLoadState(const QDomElement &parent_element)
{
    qDebug() << "DataStreamROS::xmlLoadState";

    QDomElement stamp_elem = parent_element.firstChildElement("use_header_stamp");
    _config.use_header_stamp = (stamp_elem.attribute("value") == "true");

    QDomElement discard_elem = parent_element.firstChildElement("discard_large_arrays");
    _config.discard_large_arrays = (stamp_elem.attribute("value") == "true");

    QDomElement max_elem = parent_element.firstChildElement("max_array_size");
    _config.max_array_size = (stamp_elem.attribute("value") == "true");

    _config.selected_topics.clear();
    QDomElement topic_elem = parent_element.firstChildElement("selected_topics").firstChildElement("topic");
    while (!topic_elem.isNull())
    {
        qDebug() << "Value: " << topic_elem.attribute("value");
        _config.selected_topics.push_back(topic_elem.attribute("name"));
        topic_elem = topic_elem.nextSiblingElement("topic");
    }

    return true;
}

void DataStreamROS::addActionsToParentMenu(QMenu *menu)
{

}

void DataStreamROS::saveDefaultSettings()
{
    QSettings settings;

    settings.setValue("DataStreamROS2/default_topics", _config.selected_topics);
    settings.setValue("DataStreamROS2/use_header_stamp", _config.use_header_stamp);
    settings.setValue("DataStreamROS2/discard_large_arrays", _config.discard_large_arrays);
    settings.setValue("DataStreamROS2/max_array_size", (int)_config.max_array_size);
}

void DataStreamROS::loadDefaultSettings()
{
    qDebug() << "DataStreamROS::loadDefaultSettings()";

    QSettings settings;

    _config.selected_topics = settings.value("DataStreamROS2/default_topics", false).toStringList();
    _config.use_header_stamp = settings.value("DataStreamROS2/use_header_stamp", false).toBool();
    _config.discard_large_arrays = settings.value("DataStreamROS2/discard_large_arrays", true).toBool();
    _config.max_array_size = settings.value("DataStreamROS2/max_array_size", 100).toInt();
}

