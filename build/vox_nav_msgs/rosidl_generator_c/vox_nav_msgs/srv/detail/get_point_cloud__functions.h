// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from vox_nav_msgs:srv/GetPointCloud.idl
// generated code does not contain a copyright notice

#ifndef VOX_NAV_MSGS__SRV__DETAIL__GET_POINT_CLOUD__FUNCTIONS_H_
#define VOX_NAV_MSGS__SRV__DETAIL__GET_POINT_CLOUD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "vox_nav_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "vox_nav_msgs/srv/detail/get_point_cloud__struct.h"

/// Initialize srv/GetPointCloud message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vox_nav_msgs__srv__GetPointCloud_Request
 * )) before or use
 * vox_nav_msgs__srv__GetPointCloud_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Request__init(vox_nav_msgs__srv__GetPointCloud_Request * msg);

/// Finalize srv/GetPointCloud message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Request__fini(vox_nav_msgs__srv__GetPointCloud_Request * msg);

/// Create srv/GetPointCloud message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vox_nav_msgs__srv__GetPointCloud_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
vox_nav_msgs__srv__GetPointCloud_Request *
vox_nav_msgs__srv__GetPointCloud_Request__create();

/// Destroy srv/GetPointCloud message.
/**
 * It calls
 * vox_nav_msgs__srv__GetPointCloud_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Request__destroy(vox_nav_msgs__srv__GetPointCloud_Request * msg);

/// Check for srv/GetPointCloud message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Request__are_equal(const vox_nav_msgs__srv__GetPointCloud_Request * lhs, const vox_nav_msgs__srv__GetPointCloud_Request * rhs);

/// Copy a srv/GetPointCloud message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Request__copy(
  const vox_nav_msgs__srv__GetPointCloud_Request * input,
  vox_nav_msgs__srv__GetPointCloud_Request * output);

/// Initialize array of srv/GetPointCloud messages.
/**
 * It allocates the memory for the number of elements and calls
 * vox_nav_msgs__srv__GetPointCloud_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__init(vox_nav_msgs__srv__GetPointCloud_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetPointCloud messages.
/**
 * It calls
 * vox_nav_msgs__srv__GetPointCloud_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__fini(vox_nav_msgs__srv__GetPointCloud_Request__Sequence * array);

/// Create array of srv/GetPointCloud messages.
/**
 * It allocates the memory for the array and calls
 * vox_nav_msgs__srv__GetPointCloud_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
vox_nav_msgs__srv__GetPointCloud_Request__Sequence *
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetPointCloud messages.
/**
 * It calls
 * vox_nav_msgs__srv__GetPointCloud_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__destroy(vox_nav_msgs__srv__GetPointCloud_Request__Sequence * array);

/// Check for srv/GetPointCloud message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__are_equal(const vox_nav_msgs__srv__GetPointCloud_Request__Sequence * lhs, const vox_nav_msgs__srv__GetPointCloud_Request__Sequence * rhs);

/// Copy an array of srv/GetPointCloud messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Request__Sequence__copy(
  const vox_nav_msgs__srv__GetPointCloud_Request__Sequence * input,
  vox_nav_msgs__srv__GetPointCloud_Request__Sequence * output);

/// Initialize srv/GetPointCloud message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vox_nav_msgs__srv__GetPointCloud_Response
 * )) before or use
 * vox_nav_msgs__srv__GetPointCloud_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Response__init(vox_nav_msgs__srv__GetPointCloud_Response * msg);

/// Finalize srv/GetPointCloud message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Response__fini(vox_nav_msgs__srv__GetPointCloud_Response * msg);

/// Create srv/GetPointCloud message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vox_nav_msgs__srv__GetPointCloud_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
vox_nav_msgs__srv__GetPointCloud_Response *
vox_nav_msgs__srv__GetPointCloud_Response__create();

/// Destroy srv/GetPointCloud message.
/**
 * It calls
 * vox_nav_msgs__srv__GetPointCloud_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Response__destroy(vox_nav_msgs__srv__GetPointCloud_Response * msg);

/// Check for srv/GetPointCloud message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Response__are_equal(const vox_nav_msgs__srv__GetPointCloud_Response * lhs, const vox_nav_msgs__srv__GetPointCloud_Response * rhs);

/// Copy a srv/GetPointCloud message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Response__copy(
  const vox_nav_msgs__srv__GetPointCloud_Response * input,
  vox_nav_msgs__srv__GetPointCloud_Response * output);

/// Initialize array of srv/GetPointCloud messages.
/**
 * It allocates the memory for the number of elements and calls
 * vox_nav_msgs__srv__GetPointCloud_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__init(vox_nav_msgs__srv__GetPointCloud_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetPointCloud messages.
/**
 * It calls
 * vox_nav_msgs__srv__GetPointCloud_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__fini(vox_nav_msgs__srv__GetPointCloud_Response__Sequence * array);

/// Create array of srv/GetPointCloud messages.
/**
 * It allocates the memory for the array and calls
 * vox_nav_msgs__srv__GetPointCloud_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
vox_nav_msgs__srv__GetPointCloud_Response__Sequence *
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetPointCloud messages.
/**
 * It calls
 * vox_nav_msgs__srv__GetPointCloud_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
void
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__destroy(vox_nav_msgs__srv__GetPointCloud_Response__Sequence * array);

/// Check for srv/GetPointCloud message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__are_equal(const vox_nav_msgs__srv__GetPointCloud_Response__Sequence * lhs, const vox_nav_msgs__srv__GetPointCloud_Response__Sequence * rhs);

/// Copy an array of srv/GetPointCloud messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_vox_nav_msgs
bool
vox_nav_msgs__srv__GetPointCloud_Response__Sequence__copy(
  const vox_nav_msgs__srv__GetPointCloud_Response__Sequence * input,
  vox_nav_msgs__srv__GetPointCloud_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // VOX_NAV_MSGS__SRV__DETAIL__GET_POINT_CLOUD__FUNCTIONS_H_
