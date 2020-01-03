#ifndef DATALOAD_ROS_H
#define DATALOAD_ROS_H

#include <QObject>
#include <QtPlugin>
#include <QSettings>

#include "PlotJuggler/optional.hpp"
#include <rosbag2/readers/sequential_reader.hpp>

#include "PlotJuggler/dataloader_base.h"
#include "../dialog_select_ros_topics.h"

class rosidl_message_type_support_t;

class  DataLoadROS: public DataLoader
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.robopec.PlotJuggler.ROS2DataLoader" "../dataloader.json")
    Q_INTERFACES(DataLoader)

    struct TopicMemberInfo
    {
        QString path;
        std::string name;
        uint32_t offset;
        uint8_t ros_type;
        std::unordered_map<std::string, PlotData>::iterator plot_map_iterator;
    };

    struct TopicData
    {
        std::vector<TopicMemberInfo> members;
        std::string type;
        std::shared_ptr<rosbag2_introspection_message_t> msg_buffer;
        const rosidl_message_type_support_t* typesupport;
        const rosidl_message_type_support_t* introspection_typesupport;

        nonstd::optional<uint32_t> stampSecOffset;
        nonstd::optional<uint32_t> stampNanosecOffset;
    };

public:
    DataLoadROS();

    virtual const std::vector<const char*>& compatibleFileExtensions() const override;

    virtual bool readDataFromFile(FileLoadInfo* fileload_info, PlotDataMapRef& destination) override;

    virtual const char* name() const override { return "DataLoad ROS bags"; }

    virtual ~DataLoadROS() override;

    virtual bool xmlSaveState(QDomDocument &doc, QDomElement &parent_element) const override;

    virtual bool xmlLoadState(const QDomElement &parent_element ) override;

protected:
    //std::shared_ptr<rosbag::Bag> _bag;
    std::shared_ptr<rosbag2::readers::SequentialReader> _bagReader;

private:
    void generateMessageTypesVec(std::vector<TopicMemberInfo> &membersVec,
                             const QString& path,
                             const rosidl_message_type_support_t* typeData,
                             uint32_t offset);

    std::vector<const char*> _extensions;

    DialogSelectRosTopics::Configuration _config;

    std::vector<std::pair<QString, QString>> getAndRegisterAllTopics();

    void saveDefaultSettings();

    void loadDefaultSettings();
};

#endif // DATALOAD_CSV_H
