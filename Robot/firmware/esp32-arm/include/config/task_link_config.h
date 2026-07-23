/**
 * @file task_link_config.h
 * @brief Declares arm-side policies for both task-link directions.
 *
 * One configuration receives top-owned commands from the drivetrain; the other
 * forwards scan commands from the top dispatcher to the Raspberry Pi.
 */
#pragma once

#include <robot_common/task/task_link_client.h>
#include <robot_common/task/task_link_server.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Server policy for commands received from the drivetrain coordinator. */
extern const TaskLinkServerConfig TOP_TASK_SERVER_CONFIG;
/** Client policy for scan commands forwarded to the Raspberry Pi. */
extern const TaskLinkClientConfig PI_SCAN_CLIENT_CONFIG;

#ifdef __cplusplus
}
#endif
