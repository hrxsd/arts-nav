#include "Navcore.h"
// #include <ros/ros.h>
// #include <map>
#include <iostream>
#include <string>
// #include <json/json.h>
// #include <ctime>
#include "navigation/nav_srv.h"
#include "navigation/ints.h"
#include "navigation/arx5_grasp.h"
#include <unistd.h>
#include <stdlib.h>
#include <iostream>


enum GraspMode {
    Grasp_sauce = 1,
    Grasp_seasoning,
    put_sauce,
    put_seasoning,
} GraspMode{};

// class Nav{

//     // ros::NodeHandle nh;
//     bool new_goal{false};
//     bool nav_on{false};
//     int target_idx = 0;

// public:
    
//     navigation::nav_srv navi_srv;
//     targetPose target_pose;
//     std::vector<std::string> target_type;
//     NavCore* navCore;
//     geometry_msgs::Pose2D target;
//     // std::map<char,int> mymap;

//     std::string MAP_FRAME , BASE_FOOT_PRINT;
//     void run();
//     // void readPoseFromJson(std::string );
//     Nav(const std::string& base_foot_print, std::string map_frame, std::string serial_addr, ros::NodeHandle &nh){
//         navCore = new NavCore(base_foot_print, std::move(map_frame));
        
//     };
//     ~Nav(){
//         delete navCore;
//     }

// };



// void Nav::readPoseFromJson(std::string type)
// {
// 	Json::Reader reader;/*用于按照JSON数据格式进行解析*/
// 	Json::Value root;/*用于保存JSON类型的一段数据*/
// 	std::ifstream srcFile("/home/nuc1003a/ARTS_test/src/Mapping/config/target_map.json", std::ios::binary);/*定义一个ifstream流对象，与文件demo.json进行关联*/
// 	if (!srcFile.is_open())
// 	{
// 		std::cout << "Fail to open target_map.json" << std::endl;
// 		return;
// 	}
// 	if (reader.parse(srcFile, root))
// 	{
//         // target.x = root[type][0].asDouble();
//         // target.y = root[type][1].asDouble();
//         // target.theta = root[type][2].asDouble();
//         for(int i = 0; i < 3; i++){
//             if (type == target_type[i])
//             {
//                 target_pose.targetAction = targetPose::TargetAction(i);
//                 target_pose.pose.x = root[type][0].asDouble();
//                 target_pose.pose.y = root[type][1].asDouble();
//                 target_pose.pose.theta = root[type][2].asDouble();
//                 ROS_INFO_STREAM("target_pose.targetAction: "<<target_pose.targetAction);
//                 ROS_INFO_STREAM("target_pose.x: "<<target_pose.pose.x);
//                 ROS_INFO_STREAM("target_pose.y: "<<target_pose.pose.y);
//                 ROS_INFO_STREAM("target_pose.th: "<<target_pose.pose.theta);
//             }
//         }
// 	}
// 	srcFile.close();
// }

// void Nav::run() {

//     navi_srv.request.tar_type = targetPose::shopping;
//     // std::cout<< navi_srv.request.tar_type<<std::endl;
//     bool flag = navigation_client.call(navi_srv);
//     if (flag)
//     {
//         ROS_INFO("successful to get %s!", navi_srv.response.tar_string);
//         new_goal = true;
//         nav_on = true;
//     }
//     // int nav_result = navCore->getMoveBaseActionResult();
//     // if (nav_result == NavCore::SUCCEEDED){
//     //     ROS_INFO("getting TARGET point SUCCESSFUL!!!");
//     //     target_idx++;
//     //     new_goal = true;
//     //     nav_on = true;
//     //     return;
//     // }
//     // targetPose target_pose;
//     // target_pose.targetAction = targetPose::TargetAction(2); 
//     // target_pose.pose.x = -1.5; target_pose.pose.y = 0.021;  target_pose.pose.theta = 0; 
//     // std::cout << "target_pose" << target << std::endl;
//     // target_pose.pose = target;
//     if (nav_on && new_goal){
//         navCore->setGoal(target); 
//         ROS_INFO("SET New Goal!");
//         nav_on = false;
//         new_goal = false;
//     }
    
//     ROS_INFO_STREAM("target: " << target_pose.pose);
    
//     navCore->saveGoal(target_pose);
    
// }
void actionResultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    // ROS_INFO("IN NAV CORE !!!!! callback,%d  ", msg->status_list);
    // ROS_INFO("IN NAV CORE !!!!! callback,%d  ",msg.status_list[0]);
    // actionlib_msgs::GoalStatus status;
    // uint8_t sta;
    // sta = msg->status_list[0];
    // std::cout << sta << std::endl;
     ROS_INFO("IN callback");
    // std::cout<<"resuullllllll!!!!!!!!!!"<<(*msg).status_list[0]<<std::endl;

    // 遍历所有目标点的状态
        for (const auto& status : msg->status_list) {
        // 检查当前目标点的状态是否为成功状态（状态值为3）
        ROS_INFO("????????????");
            // std::cout<<"resuullllllll!!!!!!!!!!"<<std::endl;
            if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
                ROS_INFO("Robot has reached the goal!");
                // return; // 只打印一次信息后返回
         }
        }
    

    // for (const actionlib_msgs::GoalStatus& status : msg->status_list)
    // {
    //     std::cout<<"resuullllllll!!!!!!!!!!"<<status.status<<std::endl;
    //     if (status.status == NULL)
    //     {
    //         ROS_INFO("IN NAV CORE !!!!! callback  ");
    //         // ROS_INFO("IN NAV CORE !!!!! callback,%d  ",status.status);
    //     }
    //     if (status.status == 3)//actionlib_msgs::GoalStatus::SUCCEEDED
    //     {
    //         ROS_INFO("IN NAV CORE !!!!! callback,%d  ",status.status);
    //         // moveBaseActionResult_ = MoveBaseActionResult::SUCCEEDED;
    //         ROS_INFO("MoveBase status: Goal succeeded");
    //     }
    // }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh_;
    bool newgoal{false};
    // ros::Rate loop_rate(30);

    // std::string bas frame",odom_frame,(std::string)"odom");
    // nh_.param("map_frame",map_frame,(std::string)"3dmap");
    // nh_.param("serial_addr",serial_addr,(std::string)"/dev/ttyS1");
  
    // Nav nav(base_foot_print, map_frame, serial_addr, nh_);
    
    // nav.MAP_FRAME = map_frame;
    // nav.BASE_FOOT_PRINT = base_foot_print;

    // clock_t start=0,end=0;
    // ros::ServiceClient navigation_client;
    navigation::nav_srv navi_srv;
    navigation::arx5_grasp arx_srv;
    navigation::ints addints;
    ros::ServiceClient navigation_client = nh_.serviceClient<navigation::nav_srv>("navi_service");
    ros::ServiceClient arx_client = nh_.serviceClient<navigation::arx5_grasp>("arx_server");

    ros::ServiceClient client = nh_.serviceClient<navigation::ints>("add");
    // ros::Subscriber action_result_sub = nh_.subscribe("/move_base/status", 1, actionResultCallback);
    bool serving{false};
    ros::service::waitForService("navi_service");
    ROS_INFO("#################### NAVI Server is already #####################");
    ros::service::waitForService("arx_server");
    ROS_INFO("#################### arx_server is already #####################");

    // arx_srv.request.mode = 1;
    // bool flag_grasp = arx_client.call(arx_srv);

    // ros::service::waitForService("add");
    // addints.request.num1 = 1;
    // addints.request.num2 = 2;
    // client.call(addints);
    // std::cout<<"response"<<addints.response.sum<<std::endl;
    /////////////////////////////////
        // navi_srv.request.tar_type = targetPose::shopping;
        // bool nav_flag = navigation_client.call(navi_srv);
    navi_srv.request.tar_type = 9;
    bool nav_flag_origin = navigation_client.call(navi_srv);
    // if (nav_flag){
    //     ROS_INFO("set new target");
    //     navi_srv.request.tar_type = targetPose::shopping;
    //     bool nav_flag = navigation_client.call(navi_srv);
    // }

    
    ///////////////  SHOPPING  ////////////////////
    
    navi_srv.request.tar_type = targetPose::shopping;
    bool nav_flag = navigation_client.call(navi_srv);
    if (nav_flag){
        arx_srv.request.mode = GraspMode::Grasp_sauce;
        bool flag_grasp = arx_client.call(arx_srv);
        if (flag_grasp){
            //////// GO COOKING PUT OBJ ////////////
            navi_srv.request.tar_type = targetPose::cooking;
            bool put_flag = navigation_client.call(navi_srv);
            if(put_flag) 
            {
                arx_srv.request.mode = GraspMode::put_sauce;
                bool flag_put = arx_client.call(arx_srv);
                newgoal = true;
                std::cout<<"GO GRASP ANOTHER ONE !!!!!!!"<<std::endl;
            }
        }
    }
    if (newgoal) {
        // sleep(2);
        std::cout<<"set new grasp!!!!!!!<<<<<<<<"<<std::endl;
        navi_srv.request.tar_type = targetPose::shopping;
        bool nav_flag = navigation_client.call(navi_srv);
        if (nav_flag) {
            arx_srv.request.mode = GraspMode::Grasp_seasoning;
            bool flag_grasp = arx_client.call(arx_srv);
            if (flag_grasp){
                //////// GO COOKING PUT OBJ ////////////
                navi_srv.request.tar_type = targetPose::cooking;
                bool put_flag = navigation_client.call(navi_srv);
                ROS_INFO("First put flag, %d", put_flag);
                if(put_flag)
                {
                    arx_srv.request.mode = GraspMode::put_seasoning;
                    bool flag_put = arx_client.call(arx_srv);
                    newgoal = true;
                }
                newgoal = true;
                std::cout<<"GO GRASP ANOTHER ONE !!!!!!!"<<std::endl;
            }
        }
            serving = true;
            std::cout<<"cooking!!!!!!!"<<std::endl;
        // }
    }
    if(serving){
        navi_srv.request.tar_type = targetPose::serving;
        navigation_client.call(navi_srv);
    }   

 
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