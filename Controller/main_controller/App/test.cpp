#include "loadController/common_include.h"
#include "loadController/flightController.h"
#include "loadController/node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ICRA2021_test_node");
    ros::NodeHandle nh;


    wtr::Node n;

    double massQuadcopter = 0.52;//1.65;//小电池  //中电池2
    double massPayload = 0.168;
    float length = 0.57;//0.72;//0.01;
    Mat33 II;
    II<< 0.0347, 0, 0,
            0, 0.0458, 0,
            0, 0,0.0977;
    n.controller.initializeParameter(massQuadcopter, massPayload, length, II);

    while(ros::ok()){
#ifdef VREP
        n.publishToVrep();
#endif
#ifdef PX4
        n.publishToPX4();
#endif
        ros::spinOnce();
    }
    return 0;
}

