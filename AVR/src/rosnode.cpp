#include <ros.h>
#include <std_msgs/Int32.h>
#include <stdint.h>
#include "rosnode.h"

ros::NodeHandle node;

std_msgs::Int32 position_feedback;
ros::Publisher feedback("feedback", &position_feedback);

static int32_t position = 0;
static void requested_position_cb(const std_msgs::Int32& cmd)
{
    position = cmd.data;
}

int32_t node_get_requested_position(void)
{
    return position;
}

ros::Subscriber<std_msgs::Int32> requested_position("requested_position",
                                                    requested_position_cb);
void node_init(void)
{
    node.initNode();
    node.advertise(feedback);
    node.subscribe(requested_position);
}

void node_publish_data(int32_t data)
{
    position_feedback.data = data;
    feedback.publish(&position_feedback);

    node.spinOnce(); // update ROS
}
