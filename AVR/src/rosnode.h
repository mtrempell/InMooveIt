#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_node(void);
void publish_data(int64_t data);

#ifdef __cplusplus
}
#endif

#endif // ROS_NODE_H
