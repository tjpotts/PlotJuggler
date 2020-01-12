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

typedef struct HeaderInfo
{
    uint32_t secs_offset;
    uint32_t nanosecs_offset;
    uint32_t frame_id_offset;
} HeaderInfo;

typedef struct MemberInfo
{
    std::shared_ptr<HeaderInfo> header_info;
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


const TypeInfo getTypeInfo(std::string topic_name);

std::shared_ptr<HeaderInfo> getHeaderInfo(
    const rosidl_typesupport_introspection_cpp::MessageMembers* header_typesupport,
    uint32_t offset
);

void getMemberInfo(
    const rosidl_message_type_support_t* introspection_typesupport,
    std::vector<MemberInfo>& member_info_vec,
    std::string path = "",
    std::shared_ptr<HeaderInfo> header_info = nullptr,
    uint32_t offset = 0
);

uint8_t* deserialize(std::shared_ptr<rmw_serialized_message_t> msg, TypeInfo& typeInfo);

double getMessageTime(uint8_t* deserialized_message, const HeaderInfo& header_info);
uint8_t* getMessageMember(uint8_t* deserialized_message, const MemberInfo& member_info);
double getMessageMemberNumeric(uint8_t* dserialized_message, const MemberInfo& member_info);

} // end namespace ros_parser

#endif

