#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void node_init(void);
void node_publish_data(int32_t data);
int32_t node_get_requested_position(void);

#ifdef __cplusplus
}
#endif

#endif // ROS_NODE_H
