#include "Navcore.h"
// #include <ros/ros.h>
// #include <map>
#include <iostream>
#include <string>
// #include <json/json.h>
// #include <ctime>
#include "navigation/nav_srv.h"
#include "navigation/lcm_srv.h"
#include "navigation/ints.h"
#include "navigation/arx5_grasp.h"
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

enum NavMode {
    Go_Shoping_place,
    Go_Cooking_place,
    Go_Serving_place
} NavMode{};


enum GraspMode {
    Grasp_1 = 1,
    Grasp_2 ,
    Grasp_3 ,
    Grasp_4 ,
    Grasp_5 ,
    Put_1 ,
    Put_2 ,
    Put_3 ,
    Put_4 ,
    Put_5
} GraspMode{};


int main(int argc, char* argv[]){
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh_;
    bool newgoal{false};
    ros::Rate loop_rate(30);
    navigation::nav_srv navi_srv;
    navigation::arx5_grasp arx_srv;
    navigation::lcm_srv lcm_srv;

    navigation::ints addints;
    ros::ServiceClient navigation_client = nh_.serviceClient<navigation::nav_srv>("navi_service");
    // ros::ServiceClient arx_client = nh_.serviceClient<navigation::arx5_grasp>("arx_server");
    // ros::ServiceClient lcm_client = nh_.serviceClient<navigation::lcm_srv>("lcm_server");

    bool serving{false};
    ros::service::waitForService("navi_service");
    ROS_INFO("#################### NAVI Server is already #####################");
    // ros::service::waitForService("arx_server");
    // ROS_INFO("#################### arx_server is already #####################");
    // ros::service::waitForService("lcm_server");
    // ROS_INFO("#################### lcm_server is already #####################");

    int nav_mode  = NavMode::Go_Shoping_place;
    int grasp_mode = 2;
    int put_mode = 4;
    bool nav_flag{false};
    bool flag_grasp{false};
    bool flag_put{false};
    bool all_done(false);

    while (ros::ok())
{
    switch (nav_mode)
    {
    case NavMode::Go_Shoping_place:
        ROS_INFO("############### Go_Shoping_place #################");
        navi_srv.request.tar_type = targetPose::shopping;
        nav_flag = navigation_client.call(navi_srv);
        if (nav_flag) {
            ROS_INFO("############### Grasp bottle ID%d #################", grasp_mode);
            arx_srv.request.mode = grasp_mode;
            // flag_grasp = arx_client.call(arx_srv);
            flag_grasp = true;
            if (flag_grasp) {
                ROS_INFO("############### Finish Grasp #################");
                put_mode++;
                nav_mode = NavMode::Go_Cooking_place;
            }
        }
        break;
    case NavMode::Go_Cooking_place:
        ROS_INFO("############### Go_Cooking_place #################");
        navi_srv.request.tar_type = targetPose::cooking;
        nav_flag = navigation_client.call(navi_srv);
        if (nav_flag) {
            ROS_INFO("############### Put bottle ID%d #################", put_mode);
            arx_srv.request.mode = put_mode;
            // flag_put = arx_client.call(arx_srv);
            flag_put = true;

            if (flag_put) {
                ROS_INFO("############### Finish Put #################");
                grasp_mode++;
                nav_mode = NavMode::Go_Serving_place;
            }
        }
        // if(grasp_mode > 4 ) {
        //     nav_mode = NavMode::Go_Serving_place;
        // }
        break;
    case NavMode::Go_Serving_place:
        ROS_INFO("############### Go_Serving_place #################");
        lcm_srv.request.enable = true;
        navi_srv.request.tar_type = targetPose::serving;
        all_done = navigation_client.call(navi_srv);
        // lcm_client.call(lcm_srv);
        // if (lcm_srv.response.cook_done) {
        //     navi_srv.request.tar_type = targetPose::serving;
        //     all_done = navigation_client.call(navi_srv);
        // }
    default:
        break;
    }
    if(all_done) break;
    loop_rate.sleep();
}
    /*
while (ros::ok())
{
    switch (nav_mode)
    {
    case NavMode::Go_Shoping_place:
        ROS_INFO("############### Go_Shoping_place #################");
        navi_srv.request.tar_type = targetPose::shopping;
        nav_flag = navigation_client.call(navi_srv);
        if (nav_flag) {
            ROS_INFO("############### Grasp bottle ID%d #################", grasp_mode);
            arx_srv.request.mode = grasp_mode;
            flag_grasp = arx_client.call(arx_srv);
            if (flag_grasp) {
                ROS_INFO("############### Finish Grasp #################");
                nav_mode = NavMode::Go_Cooking_place;
            }
        }
        break;
    case NavMode::Go_Cooking_place:
        ROS_INFO("############### Go_Cooking_place #################");
        navi_srv.request.tar_type = targetPose::cooking;
        nav_flag = navigation_client.call(navi_srv);
        if (nav_flag) {
            ROS_INFO("############### Put bottle ID%d #################", put_mode);
            arx_srv.request.mode = put_mode;
            flag_put = arx_client.call(arx_srv);
            if (flag_put) {
                ROS_INFO("############### Finish Put #################");
                grasp_mode++;
                nav_mode = NavMode::Go_Shoping_place;
            }
        }
        break;
    default:
        break;
    }

    loop_rate.sleep();
}
*/
    

    // navi_srv.request.tar_type = 9;
    // bool nav_flag_origin = navigation_client.call(navi_srv);


    
    // ///////////////  SHOPPING  ////////////////////
    
    // navi_srv.request.tar_type = targetPose::shopping;
    // bool nav_flag = navigation_client.call(navi_srv);
    // if (nav_flag){
    //     arx_srv.request.mode = GraspMode::Grasp_sauce;
    //     bool flag_grasp = arx_client.call(arx_srv);
    //     if (flag_grasp){
    //         //////// GO COOKING PUT OBJ ////////////
    //         navi_srv.request.tar_type = targetPose::cooking;
    //         bool put_flag = navigation_client.call(navi_srv);
    //         if(put_flag) 
    //         {
    //             arx_srv.request.mode = GraspMode::put_sauce;
    //             bool flag_put = arx_client.call(arx_srv);
    //             newgoal = true;
    //             std::cout<<"GO GRASP ANOTHER ONE !!!!!!!"<<std::endl;
    //         }
    //     }
    // }
    // if (newgoal) {
    //     // sleep(2);
    //     std::cout<<"set new grasp!!!!!!!<<<<<<<<"<<std::endl;
    //     navi_srv.request.tar_type = targetPose::shopping;
    //     bool nav_flag = navigation_client.call(navi_srv);
    //     if (nav_flag) {
    //         arx_srv.request.mode = GraspMode::Grasp_seasoning;
    //         bool flag_grasp = arx_client.call(arx_srv);
    //         if (flag_grasp){
    //             //////// GO COOKING PUT OBJ ////////////
    //             navi_srv.request.tar_type = targetPose::cooking;
    //             bool put_flag = navigation_client.call(navi_srv);
    //             ROS_INFO("First put flag, %d", put_flag);
    //             if(put_flag)
    //             {
    //                 arx_srv.request.mode = GraspMode::put_seasoning;
    //                 bool flag_put = arx_client.call(arx_srv);
    //                 newgoal = true;
    //             }
    //             newgoal = true;
    //             std::cout<<"GO GRASP ANOTHER ONE !!!!!!!"<<std::endl;
    //         }
    //     }
    //         serving = true;
    //         std::cout<<"cooking!!!!!!!"<<std::endl;
    //     // }
    // }
    // if(serving){
    //     navi_srv.request.tar_type = targetPose::serving;
    //     navigation_client.call(navi_srv);
    // }   

 
/////////////////////////////////////
    // ROS_INFO("Waiting for navigation service!");
    // while (ros::ok()) {
    //     // end = clock();
    //     // std::cout << "loop_time" << end - start << std::endl;
    //     // start = clock();

    //     geometry_msgs::Pose2D currentpose;
    //     std::string type;
    //     currentpose = nav.navCore->getCurrentPose(map_frame,base_foot_print);
    //     // std::cout<<"target_pose: "<< nav.target <<std::endl;
    //     // std::cout<<"current_pose: "<<currentpose<<std::endl;
    //     nav.run();
    //     loop_rate.sleep();
    //     ros::spinOnce();
    
//   }
ros::spin();
}