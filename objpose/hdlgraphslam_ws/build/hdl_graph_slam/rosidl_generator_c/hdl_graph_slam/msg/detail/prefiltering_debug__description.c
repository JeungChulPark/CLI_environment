// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from hdl_graph_slam:msg/PrefilteringDebug.idl
// generated code does not contain a copyright notice

#include "hdl_graph_slam/msg/detail/prefiltering_debug__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_hdl_graph_slam
const rosidl_type_hash_t *
hdl_graph_slam__msg__PrefilteringDebug__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4c, 0xd2, 0x16, 0x2f, 0x51, 0xb3, 0xde, 0x57,
      0xf9, 0xe2, 0x13, 0x1f, 0xf3, 0xd0, 0xf2, 0x6e,
      0xd5, 0x9a, 0x85, 0xa9, 0x48, 0x94, 0xba, 0x4c,
      0x2f, 0x39, 0xfd, 0x4f, 0x62, 0x2d, 0x2a, 0x00,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char hdl_graph_slam__msg__PrefilteringDebug__TYPE_NAME[] = "hdl_graph_slam/msg/PrefilteringDebug";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__header[] = "header";
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__input_point_count[] = "input_point_count";
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__transformed_point_count[] = "transformed_point_count";
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__distance_filtered_point_count[] = "distance_filtered_point_count";
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__downsampled_point_count[] = "downsampled_point_count";
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__output_point_count[] = "output_point_count";
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__callback_time_ms[] = "callback_time_ms";
static char hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__output_count[] = "output_count";

static rosidl_runtime_c__type_description__Field hdl_graph_slam__msg__PrefilteringDebug__FIELDS[] = {
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__input_point_count, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__transformed_point_count, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__distance_filtered_point_count, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__downsampled_point_count, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__output_point_count, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__callback_time_ms, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hdl_graph_slam__msg__PrefilteringDebug__FIELD_NAME__output_count, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription hdl_graph_slam__msg__PrefilteringDebug__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
hdl_graph_slam__msg__PrefilteringDebug__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hdl_graph_slam__msg__PrefilteringDebug__TYPE_NAME, 36, 36},
      {hdl_graph_slam__msg__PrefilteringDebug__FIELDS, 8, 8},
    },
    {hdl_graph_slam__msg__PrefilteringDebug__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "uint32 input_point_count\n"
  "uint32 transformed_point_count\n"
  "uint32 distance_filtered_point_count\n"
  "uint32 downsampled_point_count\n"
  "uint32 output_point_count\n"
  "\n"
  "float32 callback_time_ms\n"
  "uint64 output_count";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
hdl_graph_slam__msg__PrefilteringDebug__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hdl_graph_slam__msg__PrefilteringDebug__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 220, 220},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hdl_graph_slam__msg__PrefilteringDebug__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hdl_graph_slam__msg__PrefilteringDebug__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
