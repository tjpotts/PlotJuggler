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
#include <rosbag2/typesupport_helpers.hpp>
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

std::vector<std::pair<QString,QString>> DataLoadROS::getAndRegisterAllTopics()
{
    /*
    std::vector<std::pair<QString,QString>> all_topics;
    rosbag::View bag_view ( *_bag, ros::TIME_MIN, ros::TIME_MAX, true );

    RosIntrospectionFactory::reset();

    bool ignoreAll = false;

    for(auto& conn: bag_view.getConnections() )
    {
        const auto&  topic      =  conn->topic;
        const auto&  md5sum     =  conn->md5sum;
        const auto&  datatype   =  conn->datatype;
        const auto&  definition =  conn->msg_def;

        all_topics.push_back( std::make_pair(QString( topic.c_str()), QString( datatype.c_str()) ) );
        try {
            _ros_parser.registerSchema(
                    topic, md5sum, RosIntrospection::ROSType(datatype), definition);
            RosIntrospectionFactory::registerMessage(topic, md5sum, datatype, definition);
        }
        catch(std::exception &ex)
        {
            // there was a problem with this topic
            // a real life problem example can be found here:
            // https://github.com/rosjava/rosjava_bootstrap/issues/16
            all_topics.pop_back();

            if (ignoreAll) {
                // this is not the first error with this load and the
                // user has accepted to ignore all errors
                continue;
            }

            // prompt user to abort or continue
            QMessageBox msgBox(nullptr);
            msgBox.setWindowTitle("ROS bag error");
            msgBox.setText(QString("Topic ") +
                           QString(topic.c_str()) +
                           QString(": ") +
                           QString(ex.what()));

            QPushButton* buttonCancel = msgBox.addButton(tr("Cancel"), QMessageBox::RejectRole);
            QPushButton* buttonIgnore = msgBox.addButton(tr("Ignore"), QMessageBox::YesRole);
            QPushButton* buttonIgnoreAll = msgBox.addButton(tr("Ignore all"), QMessageBox::AcceptRole);
            msgBox.setDefaultButton(buttonIgnoreAll);
            msgBox.exec();
            if( msgBox.clickedButton() == buttonCancel)
            {
                // abort the file loading
                throw;
            }
            if( msgBox.clickedButton() == buttonIgnoreAll)
            {
                // accept this and all future errors for this load
                ignoreAll = true;
            }
        }
    }
    return all_topics;
    */
}


bool DataLoadROS::readDataFromFile(FileLoadInfo* info, PlotDataMapRef& plot_map)
{
    _bagReader = std::make_shared<rosbag2::SequentialReader>();

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
        qDebug() << QString::fromStdString(topic.name) << " : " << QString::fromStdString(topic.type);

        /*
        const rosidl_message_type_support_t * typesupport = rosbag2::get_typesupport(topic.type, rosidl_typesupport_introspection_cpp::typesupport_identifier);
        auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(typesupport->data);

        for(size_t i = 0; i < members->member_count_; i++)
        {
            auto member = members->members_[i];
            qDebug() << "Member name : " << member.name_;

            //rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
        }
        */
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
    std::unordered_map<std::string, std::vector<TopicMemberInfo>> topicsMembersData;

    std::set<std::string> topic_selected;
    for(const auto& topic: _config.selected_topics)
    {
        const std::string topicStd = topic.toStdString();
        const std::string& topicType = topicTypesByName.at(topicStd);
        topic_selected.insert( topic.toStdString() );

        // load type introspection
        const rosidl_message_type_support_t * typesupport = rosbag2::get_typesupport(topicType, rosidl_typesupport_introspection_cpp::typesupport_identifier);

        std::vector<TopicMemberInfo> v;
        generateMessageTypesVec(v, topic, typesupport, 4);
        topicsMembersData.emplace(topicStd, v);
    }

    // add topics in plot_ref
    for(auto &p : topicsMembersData)
    {
        //const std::string &topic = p.first;
        for(TopicMemberInfo &memberInfo : p.second)
        {
            // addNumeric may invalidate the iterator, but the reference to its data will still be valid
            memberInfo.plot_map_iterator = plot_map.addNumeric(memberInfo.path.toStdString());
        }
    }

    //PlotDataAny& plot_consecutive = plot_map.addUserDefined( "__consecutive_message_instances__" )->second;

    while(_bagReader->has_next())
    {
        std::shared_ptr<rosbag2::SerializedBagMessage> msg = _bagReader->read_next();

        rcutils_time_point_value_t timestamp_ns = msg->time_stamp;
        double stamp = timestamp_ns / 1e9;

        auto itToTopicMembersData = topicsMembersData.find(msg->topic_name);
        if(itToTopicMembersData != topicsMembersData.end())
        {
            for(const TopicMemberInfo& member : itToTopicMembersData->second)
            {
                double numericData = 0.;

                if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT)
                {
                    if(msg->serialized_data->buffer_length < (member.offset + sizeof(float)))
                    {
                        throw std::runtime_error("rosbag message size mismatch");
                    }
                    numericData = static_cast<double>(*reinterpret_cast<float*>(msg->serialized_data->buffer + member.offset));
                }
                else if(member.ros_type == rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE)
                {
                    if(msg->serialized_data->buffer_length < (member.offset + sizeof(double)))
                    {
                        throw std::runtime_error("rosbag message size mismatch");
                    }
                    numericData = *reinterpret_cast<double*>(msg->serialized_data->buffer + member.offset);
                }

                member.plot_map_iterator->second.pushBack(PlotData::Point(stamp, numericData));
            }
        }
    }

    info->selected_datasources = _config.selected_topics;
    return true;

    /*
    if( _bag ) _bag->close();

    _bag = std::make_shared<rosbag::Bag>();
    _ros_parser.clear();

    try{
        _bag->open( info->filename.toStdString(), rosbag::bagmode::Read );
    }
    catch( rosbag::BagException&  ex)
    {
        QMessageBox::warning(nullptr, tr("Error"),
                             QString("rosbag::open thrown an exception:\n")+
                             QString(ex.what()) );
        return false;
    }

    auto all_topics = getAndRegisterAllTopics();

    //----------------------------------

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
            return false;
        }
        _config = dialog->getResult();
    }

    saveDefaultSettings();

    _ros_parser.setUseHeaderStamp( _config.use_header_stamp );
    _ros_parser.setMaxArrayPolicy( _config.max_array_size, _config.discard_large_arrays );

    if( _config.use_renaming_rules )
    {
        _ros_parser.addRules( RuleEditing::getRenamingRules() );
    }

    //-----------------------------------
    std::set<std::string> topic_selected;
    for(const auto& topic: _config.selected_topics)
    {
        topic_selected.insert( topic.toStdString() );
    }

    QProgressDialog progress_dialog;
    progress_dialog.setLabelText("Loading... please wait");
    progress_dialog.setWindowModality( Qt::ApplicationModal );

    rosbag::View bag_view ( *_bag );

    progress_dialog.setRange(0, bag_view.size()-1);
    progress_dialog.show();

    std::vector<uint8_t> buffer;

    int msg_count = 0;

    QElapsedTimer timer;
    timer.start();


    PlotDataAny& plot_consecutive = plot_map.addUserDefined( "__consecutive_message_instances__" )->second;

    for(const rosbag::MessageInstance& msg_instance: bag_view)
    {

        const std::string& topic_name  = msg_instance.getTopic();
        double msg_time = msg_instance.getTime().toSec();
        auto data_point = PlotDataAny::Point(msg_time, nonstd::any(msg_instance) );
        plot_consecutive.pushBack( data_point );

        const std::string* key_ptr = &topic_name ;

        auto plot_pair = plot_map.user_defined.find( *key_ptr );
        if( plot_pair == plot_map.user_defined.end() )
        {
            plot_pair = plot_map.addUserDefined( *key_ptr );
        }
        PlotDataAny& plot_raw = plot_pair->second;
        plot_raw.pushBack( data_point );
        //------------------------------------------
        if( msg_count++ %100 == 0)
        {
            progress_dialog.setValue( msg_count );
            QApplication::processEvents();

            if( progress_dialog.wasCanceled() ) {
                return false;
            }
        }
        //------------------------------------------
        if( topic_selected.find( topic_name ) == topic_selected.end() )
        {
            continue;
        }

        const size_t msg_size  = msg_instance.size();
        buffer.resize(msg_size);

        ros::serialization::OStream stream(buffer.data(), buffer.size());
        msg_instance.write(stream);
        MessageRef buffer_view( buffer );
        _ros_parser.pushMessageRef( topic_name, buffer_view, msg_time );
    }

    _ros_parser.extractData(plot_map, "");

    qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

    info->selected_datasources = _config.selected_topics;
    return true;
    */
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
    _config.max_array_size = max_elem.attribute("value").toInt();

    return true;
}

void DataLoadROS::generateMessageTypesVec(std::vector<TopicMemberInfo> &membersVec, const QString &path, const rosidl_message_type_support_t *typeData, uint32_t offset)
{
    const auto* members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(typeData->data);

    for(size_t i = 0; i < members->member_count_; i++)
    {
        const rosidl_typesupport_introspection_cpp::MessageMember& member = members->members_[i];
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
            topicMemberInfo.path = path + "/" + QString::fromUtf8(member.name_);
            topicMemberInfo.offset = offset + member.offset_;
            topicMemberInfo.ros_type = member.type_id_;

            membersVec.push_back(topicMemberInfo);
            qDebug() << "adding member : " << topicMemberInfo.path << " with offset : " << topicMemberInfo.offset;
        }
        else if(member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        {
            generateMessageTypesVec(membersVec,
                               path + "/" + QString::fromUtf8(member.name_),
                               member.members_,
                               offset + member.offset_
                               );
        }
    }
}

void DataLoadROS::loadTopicToPlotMap(PlotDataMapRef &plot_map,
                                     const QString &path,
                                     const rosidl_message_type_support_t *typeData,
                                     uint32_t messageOffset,
                                     const rcutils_uint8_array_t& messageData)
{
    const auto* members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(typeData->data);

    for(size_t i = 0; i < members->member_count_; i++)
    {
        auto member = members->members_[i];
        double numericData = 0.;
        bool dataIsNumeric = false;
        if(member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT)
        {
            dataIsNumeric = true;
            uint32_t memberOffset = messageOffset + member.offset_;
            if(messageData.buffer_length < memberOffset + sizeof(float))
            {
                throw std::runtime_error("rosbag message size mismatch");
            }
            numericData = static_cast<double>(*reinterpret_cast<float*>(messageData.buffer + memberOffset));
        }
        else if(member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE)
        {
            dataIsNumeric = true;
            uint32_t memberOffset = messageOffset + member.offset_;
            if(messageData.buffer_length < memberOffset + sizeof(double))
            {
                throw std::runtime_error("rosbag message size mismatch");
            }
            numericData = *reinterpret_cast<double*>(messageData.buffer + memberOffset);
        }
        else if(member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN)
        {
            dataIsNumeric = true;
            uint32_t memberOffset = messageOffset + member.offset_;
            if(messageData.buffer_length < memberOffset + sizeof(float))
            {
                throw std::runtime_error("rosbag message size mismatch");
            }
            bool b = *reinterpret_cast<bool*>(messageData.buffer + memberOffset);
            numericData = b ? 1. : 0.;
        }
        // TODO : add integer types
        else if(member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        {
            loadTopicToPlotMap(plot_map,
                               path + "/" + QString::fromUtf8(member.name_),
                               member.members_,
                               messageOffset+member.offset_,
                               messageData);
        }

        if(dataIsNumeric)
        {
            QString numericPath = path + "/" + QString::fromUtf8(member.name_);
        }

        //qDebug() << "Member name : " << member.name_;

        //rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
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

