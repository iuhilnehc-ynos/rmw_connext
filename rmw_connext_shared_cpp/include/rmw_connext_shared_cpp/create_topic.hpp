// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMW_CONNEXT_SHARED_CPP__CREATE_TOPIC_HPP_
#define RMW_CONNEXT_SHARED_CPP__CREATE_TOPIC_HPP_

#include "rcutils/types/string_array.h"

#include "rmw/types.h"
#include "rmw_connext_shared_cpp/ndds_include.hpp"
#include "rmw_connext_shared_cpp/visibility_control.h"

namespace rmw_connext_shared_cpp
{

/// Create a DDS::Topic from a node
/**
 * \pre node must not be null.
 * \pre node must be a valid node, i.e. node->data is not nullptr.
 *
 * \param[in] node rmw node structure.
 * \param[in] topic_name ros topic name.
 * \param[in] dds_topic_name demangled topic name.
 * \param[in] dds_topic_type demangled topic type name.
 * \return created DDS::Topic, nullptr on failure.
 */
RMW_CONNEXT_SHARED_CPP_PUBLIC
DDS::Topic *
create_topic(
  const rmw_node_t * node,
  const char * topic_name,
  const char * dds_topic_name,
  const char * dds_topic_type);

/// Create a DDS::DDSContentFilteredTopic from a DDS::Topic
/**
 * \pre node must not be null.
 * \pre node must be a valid node, i.e. node->data is not nullptr.
 *
 * \param[in] node rmw node structure.
 * \param[in] cft_name ros cft topic name.
 * \param[in] topic a DDS::Topic.
 * \param[in] filter_expression a filter expression of topic.
 * \param[in] expression_parameters array of expression parameter for the filter expression.
 * \return created DDS::Topic, nullptr on failure.
 */
RMW_CONNEXT_SHARED_CPP_PUBLIC
DDS::ContentFilteredTopic *
create_content_filtered_topic(
  const rmw_node_t * node,
  const char * cft_name,
  DDS::Topic * topic,
  const char * filter_expression,
  const rcutils_string_array_t * expression_parameters);

}  // namespace rmw_connext_shared_cpp

#endif  // RMW_CONNEXT_SHARED_CPP__CREATE_TOPIC_HPP_
