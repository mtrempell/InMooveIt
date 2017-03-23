#include <stdint.h>
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include "rosnode.h"
#include "joints.h"

ros::NodeHandle node;

std_msgs::Int16MultiArray position_feedback;
ros::Publisher feedback("feedback", &position_feedback);

static int16_t positions[NUM_OF_JOINTS] = {};
static void requested_position_cb(const std_msgs::Int16MultiArray& array)
{
    for(size_t i = 0; i < array.data_length && i < NUM_OF_JOINTS; ++i) {
        positions[i] = array.data[i];
    }
}

const int16_t* node_get_requested_positions(void)
{
    return positions;
}

ros::Subscriber<std_msgs::Int16MultiArray> requested_position(
                    "requested_position", requested_position_cb);
void node_init(void)
{
    node.initNode();
    node.advertise(feedback);
    node.subscribe(requested_position);
    position_feedback.data_length = NUM_OF_JOINTS;
}

void node_publish_data(const int16_t *data)
{
    int16_t data_copy[NUM_OF_JOINTS];
    memcpy(data_copy, data, sizeof(*data)*NUM_OF_JOINTS);
    position_feedback.data = data_copy;
    feedback.publish(&position_feedback);

    node.spinOnce(); // update ROS
}
