#include <ros.h>
#include <std_msgs/Int64.h>
#include <stdint.h>
#include "rosnode.h"

ros::NodeHandle node;
std_msgs::Int64 position_feedback;
ros::Publisher feedback("feedback", &position_feedback);

void init_node(void)
{
    node.initNode();
    node.advertise(feedback);
}

void publish_data(int64_t data)
{
    position_feedback.data = data;
    feedback.publish(&position_feedback);

    node.spinOnce(); // update ROS
}

#if 0
int main(void)
{
    init_node();
    while(1) {
        delay(3000);
        publish_data(501);
        if (serialEventRun) serialEventRun();
    }
    return 0;
}
#endif
