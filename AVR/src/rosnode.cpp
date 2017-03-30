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

int16_t* node_get_requested_positions(void)
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


// waits for a rosserial connection to be establisher
void node_wait_for_connection(void)
{
    while(!node.connected()) {
        node.spinOnce();
    }

    node.loginfo("ARDUINO: Rosserial connection established");
}

char logbuff[150];
void node_log_info(const char *fmt, ...)
{
    va_list arg;
    va_start(arg, fmt);
    vsprintf(logbuff, fmt, arg);
    va_end(arg);
    node.loginfo(logbuff);
}
void node_log_err(const char *msg) {node.logerror(msg);}

void node_publish_data(int16_t *data)
{
    position_feedback.data = data;
    feedback.publish(&position_feedback);

    node.spinOnce(); // update ROS
}

