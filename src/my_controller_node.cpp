#include "MyController.h"

const float FREQ_NODE = 200.0;

int main(int argc, char **argv)
{
        ros::init(argc, argv, "my_controller_node");

        /* add your functions in this class and use it here*/
        
        //MyController my_controller();

        ros::Rate loop_rate(FREQ_NODE);
        while (ros::ok())
        {
                /* your code */

                loop_rate.sleep();
                ros::spinOnce();
        }
}
