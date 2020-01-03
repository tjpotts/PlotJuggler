#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QDir>
#include <QProcess>
#include <QSettings>
#include <QElapsedTimer>
#include <set>
#include <QDebug>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <unordered_map>
#include <rmw/rmw.h>

#include "../dialog_select_ros_topics.h"
#include "../dialog_with_itemlist.h"

DataLoadROS::DataLoadROS()
{
    _extensions.push_back( "yaml");
    loadDefaultSettings();
}

void StrCat(const std::string& a, const std::string& b,  std::string& out)
{
    out.clear();
    out.reserve(a.size() + b.size());
    out.assign(a);
    out.append(b);
}

const std::vector<const char*> &DataLoadROS::compatibleFileExtensions() const
{
    return _extensions;
}

bool DataLoadROS::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_map)
{
    auto allocator = rcutils_get_default_allocator();

    if(!_bagReader)
        _bagReader = std::make_shared<rosbag2::readers::SequentialReader>();

    QString bagDir;
    {
        QFileInfo finfo(info->filename);
        bagDir = finfo.dir().path();
    }

    rosbag2::StorageOptions storageOptions;
    storageOptions.uri = bagDir.toStdString();
    storageOptions.storage_id = "sqlite3";
    rosbag2::ConverterOptions converterOptions;
    converterOptions.input_serialization_format = "cdr";
    converterOptions.output_serialization_format = rmw_get_serialization_format();

    _bagReader->open(storageOptions, converterOptions);

    std::vector<rosbag2::TopicMetadata> metadata = _bagReader->get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topicTypesByName;

    std::vector<std::pair<QString, QString>> all_topics;
    for(const rosbag2::TopicMetadata& topic : metadata)
    {
        all_topics.push_back(std::make_pair(QString::fromStdString(topic.name), QString::fromStdString(topic.type)));
        topicTypesByName.emplace(topic.name, topic.type);
        //qDebug() << QString::fromStdString(topic.name) << " : " << QString::fromStdString(topic.type);
    }

    if( info->plugin_config.hasChildNodes() )
    {
        xmlLoadState( info->plugin_config.firstChildElement() );
    }

    if( ! info->selected_datasources.empty() )
    {
            _config.selected_topics =   info->selected_datasources;
    }
    else{
        DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topics, _config );

        if( dialog->exec() != static_cast<int>(QDialog::Accepted) )
        {
            delete dialog;
            return false;
        }

        _config = dialog->getResult();
        delete dialog;
    }

    saveDefaultSettings();

    //std::unordered_map<std::string, const rosidl_message_type_support_t*> topicsIntrospectionData; // topic and type
    std::unordered_map<std::string, TopicData> topicsMembersData;

    std::set<std::string> topic_selected;
    for(const auto& topic: _config.selected_topics)
    {
        const std::string topicStd = topic.toStdString();
        const std::string& topicType = topicTypesByName.at(topicStd);
        topic_selected.insert( topic.toStdString() );

        // load type introspection
        const rosidl_message_type_support_t * introspection_typesupport = rosbag2::get_typesupport(topicType, rosidl_typesupport_introspection_cpp::typesupport_identifier);
        const rosidl_message_type_support_t * typesupport = rosbag2::get_typesupport(topicType, rosidl_typesupport_cpp::typesupport_identifier);


        TopicData td;
        td.msg_buffer = rosbag2::allocate_introspection_message(introspection_typesupport, &allocator);
        td.introspection_typesupport = introspection_typesupport;
        td.typesupport = typesupport;
        td.type = topicType;
        generateMessageTypesVec(td.members, topic, introspection_typesupport, 0);

        // check for stamp
        if(_config.use_header_stamp)
        {
            for(const auto& member : td.members)
            {
                if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32 &&
                        (member.path == topic + "/header/stamp/sec" || member.path == topic + "/stamp/sec"))
                {
                    td.stampSecOffset = member.offset;
                }

                if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32 &&
                        (member.path == topic + "/header/stamp/nanosec" || member.path == topic + "/stamp/nanosec"))
                {
                    td.stampNanosecOffset = member.offset;
                }
            }
        }

        topicsMembersData.emplace(topicStd, td);
    }

    // add topics in plot_ref
    for(auto &p : topicsMembersData)
    {
        //const std::string &topic = p.first;
        for(TopicMemberInfo &memberInfo : p.second.members)
        {
            // addNumeric may invalidate the iterator, but the reference to its data will still be valid
            memberInfo.plot_map_iterator = plot_map.addNumeric(memberInfo.path.toStdString());
        }
    }

    while(_bagReader->has_next())
    {
        std::shared_ptr<rosbag2::SerializedBagMessage> msg = _bagReader->read_next();

        auto itToTopicMembersData = topicsMembersData.find(msg->topic_name);
        if(itToTopicMembersData != topicsMembersData.end())
        {
            TopicData &td = itToTopicMembersData->second;
            auto deserializedMsgData = td.msg_buffer->message;
            auto deserialize_ret = rmw_deserialize(msg->serialized_data.get(), td.typesupport, deserializedMsgData);
            if(deserialize_ret != RMW_RET_OK)
            {
                throw std::runtime_error("Failed to deserialize topic " + itToTopicMembersData->first + " error : " + std::to_string(deserialize_ret));
            }
            // cast void* to uint8_t*, to avoid compiler warnings
            uint8_t* deserializedMsgDataAsUInt8 = static_cast<uint8_t*>(deserializedMsgData);

            double stamp;
            if(_config.use_header_stamp && td.stampSecOffset && td.stampNanosecOffset)
            {
                int32_t stamp_sec = *reinterpret_cast<int32_t*>(deserializedMsgDataAsUInt8 + td.stampSecOffset.value());
                uint32_t stamp_nanosec = *reinterpret_cast<uint32_t*>(deserializedMsgDataAsUInt8 + td.stampNanosecOffset.value());

                int64_t stamp_total_nanosec = static_cast<int64_t>(stamp_sec)*1000000000 + static_cast<int64_t>(stamp_nanosec);
                stamp = stamp_total_nanosec / 1e9;
            }
            else
            {
                rcutils_time_point_value_t timestamp_ns = msg->time_stamp;
                stamp = timestamp_ns / 1e9;
            }

            for(const TopicMemberInfo& member : itToTopicMembersData->second.members)
            {
                double numericData = 0.;

                if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT)
                {
                    numericData = static_cast<double>(*reinterpret_cast<float*>(deserializedMsgDataAsUInt8 + member.offset));
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE)
                {
                    numericData = *reinterpret_cast<double*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8)
                {
                    numericData = *reinterpret_cast<int8_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16)
                {
                    numericData = *reinterpret_cast<int16_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32)
                {
                    numericData = *reinterpret_cast<int32_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64)
                {
                    numericData = *reinterpret_cast<int64_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8)
                {
                    numericData = *reinterpret_cast<uint8_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16)
                {
                    numericData = *reinterpret_cast<uint16_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32)
                {
                    numericData = *reinterpret_cast<uint32_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64)
                {
                    numericData = *reinterpret_cast<uint64_t*>(deserializedMsgDataAsUInt8 + member.offset);
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN)
                {
                    // force boolean data to be converted to 0 or 1
                    numericData = (*reinterpret_cast<bool*>(deserializedMsgDataAsUInt8 + member.offset)) ? 1 : 0;
                }

                member.plot_map_iterator->second.pushBack(PlotData::Point(stamp, numericData));
            }
        }
    }

    info->selected_datasources = _config.selected_topics;
    return true;
}

DataLoadROS::~DataLoadROS()
{

}

bool DataLoadROS::xmlSaveState(QDomDocument &doc, QDomElement &plugin_elem) const
{
    QDomElement stamp_elem = doc.createElement("use_header_stamp");
    stamp_elem.setAttribute("value", _config.use_header_stamp ? "true" : "false");
    plugin_elem.appendChild( stamp_elem );

    QDomElement discard_elem = doc.createElement("discard_large_arrays");
    discard_elem.setAttribute("value", _config.discard_large_arrays ? "true" : "false");
    plugin_elem.appendChild( discard_elem );

    QDomElement max_elem = doc.createElement("max_array_size");
    max_elem.setAttribute("value", QString::number(_config.max_array_size));
    plugin_elem.appendChild( max_elem );

    return true;
}

bool DataLoadROS::xmlLoadState(const QDomElement &parent_element)
{
    QDomElement stamp_elem = parent_element.firstChildElement( "use_header_stamp" );
    _config.use_header_stamp = ( stamp_elem.attribute("value") == "true");

    QDomElement discard_elem = parent_element.firstChildElement( "discard_large_arrays" );
    _config.discard_large_arrays = ( discard_elem.attribute("value") == "true");

    QDomElement max_elem = parent_element.firstChildElement( "max_array_size" );
    _config.max_array_size = static_cast<size_t>(max_elem.attribute("value").toInt());

    return true;
}

void DataLoadROS::generateMessageTypesVec(std::vector<TopicMemberInfo> &membersVec, const QString &path, const rosidl_message_type_support_t *typeData, uint32_t offset)
{
    const auto* members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(typeData->data);

    for(size_t i = 0; i < members->member_count_; i++)
    {
        const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[i];
        if(member.is_array_)
            continue; // unsupported

        if(     member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8 ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16 ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32 ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64 ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8 ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16 ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32 ||
                member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64
           )
        {
            TopicMemberInfo topicMemberInfo;
            topicMemberInfo.name = std::string(member.name_);
            topicMemberInfo.path = path + "/" + QString::fromUtf8(member.name_);
            topicMemberInfo.offset = offset + member.offset_;
            topicMemberInfo.ros_type = member.type_id_;

            membersVec.push_back(topicMemberInfo);
            //qDebug() << "adding member : " << topicMemberInfo.path << " with offset : " << topicMemberInfo.offset;
        }
        else if(member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        {
            generateMessageTypesVec(membersVec,
                               path + "/" + QString::fromUtf8(member.name_),
                               member.members_,
                               offset + member.offset_
                               );
        }
        else if(member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING)
        {
            // TODO
            /*
            TopicMemberInfo topicMemberInfo;
            topicMemberInfo.path = path + "/" + QString::fromUtf8(member.name_);
            topicMemberInfo.offset = offset + member.offset_;
            topicMemberInfo.ros_type = member.type_id_;
            */
        }
    }
}

void DataLoadROS::saveDefaultSettings()
{
    QSettings settings;

    settings.setValue("DataLoadROS/default_topics", _config.selected_topics);
    settings.setValue("DataLoadROS/use_header_stamp", _config.use_header_stamp);
    settings.setValue("DataLoadROS/max_array_size", (int)_config.max_array_size);
    settings.setValue("DataLoadROS/discard_large_arrays", _config.discard_large_arrays);
}


void DataLoadROS::loadDefaultSettings()
{
    QSettings settings;

    _config.selected_topics      = settings.value("DataLoadROS/default_topics", false ).toStringList();
    _config.use_header_stamp     = settings.value("DataLoadROS/use_header_stamp", false ).toBool();
    _config.max_array_size       = settings.value("DataLoadROS/max_array_size", 100 ).toInt();
    _config.discard_large_arrays = settings.value("DataLoadROS/discard_large_arrays", true ).toBool();
}

