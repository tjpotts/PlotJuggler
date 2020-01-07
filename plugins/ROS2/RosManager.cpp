#include "RosManager.h"

#include <QDebug>

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

void RosManager::subscribe(QString topic)
{
    qDebug() << "Subscribing to topic: " << topic << endl;
}

