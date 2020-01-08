#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include "rclcpp/rclcpp.hpp"

#include <QHash>
#include <QThread>
#include <QTimer>

#include "generic_subscription.hpp"

typedef QHash<QString, QString> QRosTopicList;
Q_DECLARE_METATYPE(QRosTopicList);

Q_DECLARE_METATYPE(std::shared_ptr<rmw_serialized_message_t>);

struct TopicData
{

};

// Singleton class which manages a ros2 node and provides interfaces
// to query topics available and subscribe to topics
class RosManager : public QObject
{
    Q_OBJECT

public:
    RosManager();
    ~RosManager();

public slots:
    void startTopicListListener();
    void stopTopicListListener();
    void clearSubscriptions();
    void subscribe(QString topic);

signals:
    void topicListUpdated(QRosTopicList topic_list);
    void messageReceived(QString topic, std::shared_ptr<rmw_serialized_message_t> msg);

private:
    QThread _thread;
    std::shared_ptr<rclcpp::Context> _context;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
    std::shared_ptr<rclcpp::Node> _node;

    QTimer _spin_timer;
    QTimer _topic_list_timer;

    std::string getTopicTypeName(std::string topic);

    QHash<QString, std::shared_ptr<rosbag2_transport::GenericSubscription>> _subscriptions;

    void rosMessageCallback(QString topic, std::shared_ptr<rmw_serialized_message_t> msg);

private slots:
    void run();
    void quit();
    void updateTopicList();
    void spin();
};

#endif

