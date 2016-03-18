#include <ros/ros.h>
#include <hector_stairs_planner_disruption_test/hector_stairs_planner_disruption_test.h>

namespace hector_stairs_planner_disruption_test{
HectorStairsPlannerDisruptionTest::HectorStairsPlannerDisruptionTest(){
    ROS_INFO ("disruption node started");

    //load params
    nh_.param("desruptionPosX", desruptionPosX_, 0.0);
    nh_.param("desruptionPosY", desruptionPosY_, 0.0);
    nh_.param("toqueZNm", toqueZNm_, 50.0);
    nh_.param("forceXN", forceXN_, 100.0);
    nh_.param("duration", duration_, 0.5);

    ROS_INFO("desruption position x: %f;    y: %f", desruptionPosX_, desruptionPosY_);

    body_wrench_srv_client_=nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    robot_position_srv_client_= nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
//    robot_pos_sub_=nh_.subscribe("/robot_pose", 10, &HectorStairsPlannerDisruptionTest::robotPoseCB, this);
//    carrot_pos_sub_=nh_.subscribe("/carrot", 10, &HectorStairsPlannerDisruptionTest::carrotPoseCB, this);


    float robot_pos_x;
    float robot_pos_y;
    float pos_tolerance_x=0.5;
    float pos_tolerance_y=0.5;
    bool disrupted=false;
//    float maxXDeviation=0;



    while(1){
        gazebo_msgs::GetLinkState robot_link_position_msg;
        robot_link_position_msg.request.link_name="base_link";

        if(robot_position_srv_client_.call(robot_link_position_msg)){
            //                ROS_INFO("robot position servicecall succsesfull");
            robot_pos_x=robot_link_position_msg.response.link_state.pose.position.x;
            robot_pos_y=robot_link_position_msg.response.link_state.pose.position.y;
            //                ROS_INFO("robot position x: %f;    y: %f", robot_pos_x, robot_pos_y);
        }


        if(disrupted==false){
            if(fabs(robot_pos_x - desruptionPosX_)<pos_tolerance_x && fabs(robot_pos_y - desruptionPosY_)<pos_tolerance_y){
                gazebo_msgs::ApplyBodyWrench body_wrench_msg;
                body_wrench_msg.request.body_name="base_link";
                body_wrench_msg.request.wrench.force.x=forceXN_;
                body_wrench_msg.request.wrench.torque.z=toqueZNm_;
                ros::Duration duration(duration_);

                body_wrench_msg.request.duration=duration;

                if (body_wrench_srv_client_.call(body_wrench_msg))
                {
                    ROS_INFO("Succses: %i", body_wrench_msg.response.success);
                }
                else
                {
                    ROS_INFO("FAILD!!! Succses: %i", body_wrench_msg.response.success);
                }

                ROS_INFO ("disruption done");
                disrupted=true;
            }
        }
    }
}

HectorStairsPlannerDisruptionTest::~HectorStairsPlannerDisruptionTest()
{}
}
