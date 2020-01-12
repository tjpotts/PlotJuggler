#include "ros_parser.h"

#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

#include <QDebug>

namespace ros_parser
{

const TypeInfo getTypeInfo(std::string type_name)
{
    auto typesupport = rosbag2::get_typesupport(type_name, rosidl_typesupport_cpp::typesupport_identifier);
    auto introspection_typesupport = rosbag2::get_typesupport(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    auto allocator = rcutils_get_default_allocator();

    TypeInfo type_info;

    type_info.type_name = type_name;
    type_info.typesupport = typesupport;
    type_info.introspection_typesupport = introspection_typesupport;
    type_info.msg_buffer = rosbag2::allocate_introspection_message(introspection_typesupport, &allocator);

    type_info.members = std::vector<MemberInfo>();
    getMemberInfo(introspection_typesupport, type_info.members);

    return type_info;
}

std::shared_ptr<HeaderInfo> getHeaderInfo(
    const rosidl_typesupport_introspection_cpp::MessageMembers* header_typesupport,
    uint32_t offset
) {
    // TODO: Add a check on the incoming header_typesupport to ensure it is actually a header

    auto header_info = std::make_shared<HeaderInfo>();
    auto stamp_typesupport = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(header_typesupport->members_[0].members_->data);
    auto stamp_offset = offset + header_typesupport->members_[0].offset_;

    header_info->secs_offset = stamp_offset + stamp_typesupport->members_[0].offset_;
    header_info->nanosecs_offset = stamp_offset + stamp_typesupport->members_[1].offset_;
    header_info->frame_id_offset = offset + header_typesupport->members_[1].offset_;

    return header_info;
}

void getMemberInfo(
    const rosidl_message_type_support_t* introspection_typesupport,
    std::vector<MemberInfo>& member_info_vec,
    std::string path,
    std::shared_ptr<HeaderInfo> header_info,
    uint32_t offset
) {
    auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection_typesupport->data);

    // Get the header information for the message if it contains one
    for (int i = 0; i < members->member_count_; ++i)
    {
        auto member = members->members_[i];
        if (member.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        {
            // Not a message
            continue;
        }

        auto member_typesupport = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
        auto member_type_name = member_typesupport->message_name_;
        auto member_namespace = member_typesupport->message_namespace_;
        if (std::string(member_type_name) == "Header" && std::string(member_namespace) == "std_msgs::msg")
        {
            header_info = getHeaderInfo(member_typesupport, offset + member.offset_);
        }
    }

    for (int i = 0; i < members->member_count_; ++i)
    {
        auto member = members->members_[i];
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        {
            // TODO: Handle dynamically-sized arrays
            // TODO: Handle non-message arrays (e.g. float32[])
            auto new_path = std::string(path);
            if (new_path.size() > 0)
            {
                new_path = new_path + "/";
            }
            new_path = new_path + std::string(member.name_);

            if (member.is_array_)
            {
                auto array_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
                for (int j = 0; j < member.array_size_; ++j)
                {
                    auto array_member_path = new_path + "/" + std::to_string(j);
                    auto array_member_offset = offset + member.offset_ + array_members->size_of_ * j;
                    getMemberInfo(member.members_, member_info_vec, array_member_path, header_info, array_member_offset);
                }
            }
            else
            {
                getMemberInfo(member.members_, member_info_vec, new_path, header_info, offset + member.offset_);
            }
        }
        else
        {
            MemberInfo member_info;
            member_info.header_info = header_info;
            member_info.path = path;
            member_info.name = std::string(member.name_);
            member_info.offset = offset + member.offset_;
            member_info.ros_type = member.type_id_;
            member_info_vec.push_back(member_info);
        }
    }
}

uint8_t* deserialize(std::shared_ptr<rmw_serialized_message_t> msg, TypeInfo& typeInfo)
{
    // TODO: check return value of rmw_deserialize
    rmw_deserialize(msg.get(), typeInfo.typesupport, typeInfo.msg_buffer->message);

    return (uint8_t*)typeInfo.msg_buffer->message;
}

double getMessageTime(uint8_t* deserialized_message, const HeaderInfo& header_info)
{
    auto header_secs = *reinterpret_cast<int32_t*>(deserialized_message + header_info.secs_offset);
    auto header_nanosecs = *reinterpret_cast<uint32_t*>(deserialized_message + header_info.nanosecs_offset);

    return header_secs + (header_nanosecs * 1e-9);
}

uint8_t* getMessageMember(uint8_t* deserialized_message, const MemberInfo& member_info)
{
    return deserialized_message + member_info.offset;
}

double getMessageMemberNumeric(uint8_t* deserialized_message, const MemberInfo& member_info)
{
    double data = 0.0;
    auto member = getMessageMember(deserialized_message, member_info);

    switch (member_info.ros_type)
    {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            data = *reinterpret_cast<float*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            data = *reinterpret_cast<double*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
            data = *reinterpret_cast<char*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
            data = *reinterpret_cast<bool*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            data = *reinterpret_cast<uint8_t*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            data = *reinterpret_cast<int8_t*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            data = *reinterpret_cast<uint16_t*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            data = *reinterpret_cast<int16_t*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            data = *reinterpret_cast<uint32_t*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            data = *reinterpret_cast<int32_t*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            data = *reinterpret_cast<uint64_t*>(member);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            data = *reinterpret_cast<int64_t*>(member);
            break;
        // Unsupported types
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        default:
            break;
    }

    return data;
}

} // end namespace ros_parser

