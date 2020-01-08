#include "ros_parser.h"

#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace ros_parser
{

TypeInfo getTypeInfo(std::string type_name)
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

void getMemberInfo(
    const rosidl_message_type_support_t* introspection_typesupport,
    std::vector<MemberInfo>& member_info_vec,
    std::string path,
    uint32_t offset
) {
    auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection_typesupport->data);

    for (int i = 0; i < members->member_count_; ++i)
    {
        auto member = members->members_[i];
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        {
            getMemberInfo(member.members_, member_info_vec, path + "/" + std::string(member.name_), member.offset_);
        }
        else
        {
            MemberInfo member_info;
            member_info.path = path;
            member_info.name = std::string(member.name_);
            member_info.offset = offset + member.offset_;
            member_info.ros_type = member.type_id_;
            member_info_vec.push_back(member_info);
        }
    }
}

} // end namespace ros_parser

