#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void node_init(void);
void node_publish_data(int16_t *data);
int16_t *node_get_requested_positions(void);
void node_wait_for_connection(void);

void node_log_info(const char *fmt, ...);
void node_log_err(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // ROS_NODE_H
