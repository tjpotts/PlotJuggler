#ifndef ROS_MANAGER_H
#define ROS_MANAGER_H

#include "rclcpp/rclcpp.hpp"

#include <QHash>
#include <QThread>
#include <QTimer>

typedef QHash<QString, QString> QRosTopicList;
Q_DECLARE_METATYPE(QRosTopicList);

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
    void subscribe(QString topic);

signals:
    void topicListUpdated(QRosTopicList topic_list);
    void messageReceived(QString topic);

private:
    QThread _thread;
    std::shared_ptr<rclcpp::Context> _context;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
    std::shared_ptr<rclcpp::Node> _node;

    QTimer _spin_timer;
    QTimer _topic_list_timer;

private slots:
    void run();
    void quit();
    void updateTopicList();
    void spin();
};

#endif

