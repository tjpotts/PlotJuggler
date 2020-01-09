#ifndef ROS_PARSER_H
#define ROS_PARSER_H

#include <string>
#include <unordered_map>
#include <memory>
#include <vector>

#include "rmw/rmw.h"
#include "rmw/types.h"

#include "rosbag2/typesupport_helpers.hpp"
#include "rosbag2/types/introspection_message.hpp"

#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace ros_parser
{

typedef struct MemberInfo
{
    std::string path;
    std::string name;
    uint32_t offset;
    uint8_t ros_type;
} MemberInfo;

typedef struct TypeInfo
{
    std::string type_name;
    const rosidl_message_type_support_t* typesupport;
    const rosidl_message_type_support_t* introspection_typesupport;
    std::shared_ptr<rosbag2_introspection_message_t> msg_buffer;
    std::vector<MemberInfo> members;
} TypeInfo;


TypeInfo getTypeInfo(std::string topic_name);

void getMemberInfo(
    const rosidl_message_type_support_t* introspection_typesupport,
    std::vector<MemberInfo>& member_info_vec,
    std::string path = "",
    uint32_t offset = 0
);

uint8_t* deserialize(std::shared_ptr<rmw_serialized_message_t> msg, TypeInfo& typeInfo);

uint8_t* getMessageMember(uint8_t* deserialized_message, const MemberInfo& member_info);
double getMessageMemberNumeric(uint8_t* dserialized_message, const MemberInfo& member_info);

} // end namespace ros_parser

#endif

