#ifndef DATALOAD_ROS_H
#define DATALOAD_ROS_H

#include <QObject>
#include <QtPlugin>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "PlotJuggler/dataloader_base.h"
#include "../dialog_select_ros_topics.h"
#include "../RosMsgParsers/ros_parser.h"

class  DataLoadROS: public DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.icarustechnology.PlotJuggler.DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

public:
    DataLoadROS();

    virtual const std::vector<const char*>& compatibleFileExtensions() const override;

    virtual bool readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& destination) override;

    virtual const char* name() const override { return "DataLoad ROS bags"; }

    virtual ~DataLoadROS() override;

    virtual bool xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const override;

    virtual bool xmlLoadState(const QDomElement &parent_element ) override;

protected:
    void loadSubstitutionRule(QStringList all_topic_names);
    std::shared_ptr<rosbag::Bag> _bag;

private:

    std::vector<const char*> _extensions;

    DialogSelectRosTopics::Configuration _config;

    std::vector<std::pair<QString, QString>> getAllTopics(const rosbag::Bag *bag,
                                                          std::map<std::string, RosMessageParser> &parsers);

    void saveDefaultSettings();

    void loadDefaultSettings();
};

#endif // DATALOAD_CSV_H
