#include "RosManager.h"

#include <functional>

#include <QDebug>

#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosbag2/typesupport_helpers.hpp"

using namespace std::placeholders;

RosManager::RosManager() :
    _thread(),
    _executor(nullptr),
    _node(nullptr),
    _spin_timer(this),
    _topic_list_timer(this)
{
    qRegisterMetaType<QRosTopicList>();

    setParent(0);
    moveToThread(&_thread);

    connect(&_thread,&QThread::started,
            this,&RosManager::run);
    connect(&_thread,&QThread::finished,
            this,&RosManager::quit);

    connect(&_spin_timer,&QTimer::timeout,
            this,&RosManager::spin);
    connect(&_topic_list_timer,&QTimer::timeout,
            this,&RosManager::updateTopicList);

    _thread.start();
}

RosManager::~RosManager()
{
    _thread.quit();
    _thread.wait();
}

void RosManager::run()
{
    _context = std::make_shared<rclcpp::Context>();
    _context->init(0, nullptr);

    auto exec_args = rclcpp::executor::ExecutorArgs();
    exec_args.context = _context;
    _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exec_args);

    auto node_opts = rclcpp::NodeOptions();
    node_opts.context(_context);
    _node = std::make_shared<rclcpp::Node>("plotjuggler",node_opts);

    _executor->add_node(_node);
    _spin_timer.start();
}

void RosManager::spin()
{
    _executor->spin_once(std::chrono::milliseconds(1));
}

void RosManager::quit()
{
    _spin_timer.stop();
    _topic_list_timer.stop();
    _executor->remove_node(_node);
    _context->shutdown("");
}

void RosManager::startTopicListListener()
{
    updateTopicList();
    _topic_list_timer.start(1000);
}

void RosManager::updateTopicList()
{
    auto topic_list = _node->get_topic_names_and_types();
    QRosTopicList topic_list_out;
    for (auto topic : topic_list) {
        // TODO: Handle topics with multiple types
        auto topic_name = QString::fromStdString(topic.first);
        auto type_name = QString::fromStdString(topic.second[0]);
        topic_list_out[topic_name] = type_name;
    }

    emit topicListUpdated(topic_list_out);
}

void RosManager::stopTopicListListener()
{
    _topic_list_timer.stop();
}

std::string RosManager::getTopicTypeName(std::string topic)
{
    auto topic_list = _node->get_topic_names_and_types();
    for (auto t : topic_list)
    {
        if (t.first == topic)
        {
            return t.second[0];
        }
    }
    return "";
}

void RosManager::clearSubscriptions()
{
    qDebug() << "Clearing subscriptions" << endl;
    // The node only maintains a weak pointer to subscriptions, so getting rid of
    // the shared pointers here will cause them to be removed
    _subscriptions.clear();
}

void RosManager::subscribe(QString topic)
{
    qDebug() << "Subscribing to topic: " << topic << endl;

    if (_subscriptions.find(topic) != _subscriptions.end())
    {
        return;
    }

    auto type_name = getTopicTypeName(topic.toStdString());
    auto type_support = rosbag2::get_typesupport(type_name, rosidl_typesupport_cpp::typesupport_identifier);
    auto bound_callback = std::bind(&RosManager::rosMessageCallback, this, topic, _1);

    auto subscription = std::make_shared<rosbag2_transport::GenericSubscription>(
        _node->get_node_base_interface().get(),
        *type_support,
        topic.toStdString(),
        bound_callback
    );

    _subscriptions[topic] = subscription;
    _node->get_node_topics_interface()->add_subscription(subscription, nullptr);
}

void RosManager::rosMessageCallback(QString topic, std::shared_ptr<rmw_serialized_message_t> msg)
{
    qDebug() << "Received message for topic: " << topic << endl;
}

