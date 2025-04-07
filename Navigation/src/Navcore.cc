#include "Navcore.h"


NavCore::NavCore(std::string base_foot_print,std::string map_frame):BASE_FOOT_PRINT_(std::move(base_foot_print)),MAP_FRAME_(std::move(map_frame))
{
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

    moveBaseClient = new MoveBaseClient("move_base",true);
    moveBaseClient->waitForServer(ros::Duration(6.0));

    if(!(isMoveBaseClientConnected_ = moveBaseClient->isServerConnected()))
        ROS_ERROR_STREAM("MoveBase ActionServer Failed");
    else
        ROS_INFO_STREAM("MoveBase ActionServer Connected");

    client =nh.serviceClient<std_srvs::Empty>("clear_costmaps");
    
    // action_result_sub = nh.subscribe("/move_base/status", 1, &NavCore::actionResultCallback,this);
    action_result_sub = nh.subscribe("/move_base/result", 10, &NavCore::actionResultCallback,this);
    // ros::Subscriber sub = nh.subscribe("/move_base/status", 10, statusCallback);
}
NavCore::~NavCore()
{
    delete moveBaseClient;
    delete tfListener_;
}

// void NavCore::actionResultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
// {
//     // ROS_INFO("IN NAV CORE !!!!! callback,%d  ", msg->status_list);
    
//     // ROS_INFO("IN NAV CORE !!!!! callback,%d  ",msg.status_list[0]);
//     for (auto& status : msg->status_list)
//     {
//         // std::cout<<"resuullllllll!!!!!!!!!!"<<status.status<<std::endl;
//         // if (status.status == NULL)
//         // {
//         //     // ROS_INFO("IN NAV CORE !!!!! callback  ");
//         //     // ROS_INFO("IN NAV CORE !!!!! callback,%d  ",status.status);
//         // } 
        
//         if (status.status == 3)//actionlib_msgs::GoalStatus::SUCCEEDED
//         {
//             ROS_INFO("IN NAV CORE !!!!! callback,%d  ",status.status);
//             moveBaseActionResult_ = MoveBaseActionResult::SUCCEEDED;
//             ROS_INFO("MoveBase status: Goal succeeded");
//         }
//     }
// }



void NavCore::actionResultCallback(const move_base_msgs::MoveBaseActionResult &msg)
// void NavCore::actionResultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    boost::unique_lock<boost::shared_mutex> writeLock(action_result_mutex_);
    std::cout << "movebase  result" << msg.status.status << std::endl;
    
    switch (msg.status.status)
    {
        case move_base_msgs::MoveBaseActionResult::_status_type::PENDING:
            moveBaseActionResult_=MoveBaseActionResult::PENDDING;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::ACTIVE:
            moveBaseActionResult_ = MoveBaseActionResult::ACTIVE;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::PREEMPTED:
            moveBaseActionResult_ = MoveBaseActionResult::PREEMPTED;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::SUCCEEDED:
            moveBaseActionResult_ = MoveBaseActionResult::SUCCEEDED;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::ABORTED:
            moveBaseActionResult_ = MoveBaseActionResult::ABORTED;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::REJECTED:
            moveBaseActionResult_ = MoveBaseActionResult::REJECTED;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::PREEMPTING:
            moveBaseActionResult_ = MoveBaseActionResult::PREEMPTING;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::RECALLING:
            moveBaseActionResult_ = MoveBaseActionResult::RECALLING;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::RECALLED:
            moveBaseActionResult_ = MoveBaseActionResult::RECALLED;
            break;
        case move_base_msgs::MoveBaseActionResult::_status_type::LOST:
            moveBaseActionResult_ = MoveBaseActionResult::LOST;
            break;
        default:
            moveBaseActionResult_ = MoveBaseActionResult::EMPTY;
            break;
    }
}

/*
void NavCore::saveGoal(targetPose& pose)
{
    geometry_msgs::Pose2D current_pose{};
 
    std::ofstream output_file;
    
    output_file.open(FILE_NAME_.c_str(), std::ios::out | std::ios::trunc);
    if(output_file.is_open())
        {
            output_file << pose.targetAction << " " << pose.pose.x << " " << pose.pose.y << " " << pose.pose.theta << std::endl;   
            output_file.close();
            // std::cout<<"finish"<<std::endl;
        }       

    if(NavCore::getCurrentPose_bool(current_pose))
    {
        // ROS_INFO_STREAM("cuuu: " << current_pose);
        pose.pose = current_pose;
        static bool first_time=true;
        if(first_time)
        {
            output_file.open(FILE_NAME_.c_str(), std::ios::out | std::ios::trunc);
            first_time=false;
        }
        else
            output_file.open(FILE_NAME_.c_str(), std::ios::out | std::ios::app);
        if(output_file.is_open())
        {
            output_file << (int)pose.targetAction << " " << current_pose.x << " " << current_pose.y << " " << current_pose.theta << std::endl;   
            output_file.close();
            ROS_INFO("write done with%d %lf  %lf and %lf",(int)pose.targetAction,current_pose.x,current_pose.y,current_pose.theta);
        }
        else
            ROS_ERROR("Can not create or open the pointed file!!!!");
    }
    else
        ROS_ERROR("Can not check out the transform!!!! may the tf goes wrong or no localization!!!");
}


bool NavCore::getCurrentPose_bool(geometry_msgs::Pose2D & current_pose)
{
    geometry_msgs::TransformStamped transformStamped{};
    try
    {
        transformStamped = tfBuffer_.lookupTransform(MAP_FRAME_, BASE_FOOT_PRINT_,
                                                     ros::Time(0),ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
        return false;
    }

    current_pose.x=transformStamped.transform.translation.x;
    current_pose.y=transformStamped.transform.translation.y;
    current_pose.theta=tf::getYaw(transformStamped.transform.rotation);
    return true;
}
*/

const geometry_msgs::Pose2D & NavCore::getCurrentPose(const std::string &target_frame, const std::string &source_frame)
{
    geometry_msgs::TransformStamped transformStamped{};
    try
    {
        transformStamped = tfBuffer_.lookupTransform(target_frame, source_frame,
                                                     ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
    current_pose_.x = transformStamped.transform.translation.x;
    current_pose_.y = transformStamped.transform.translation.y;
    current_pose_.theta = tf::getYaw(transformStamped.transform.rotation);
    return current_pose_;
}

bool NavCore::isGoalPassed(const geometry_msgs::Pose2D &goal_pose)
{
    getCurrentPose(MAP_FRAME_,BASE_FOOT_PRINT_);
    ROS_INFO_STREAM("current pose: x is "<<current_pose_.x<<" y is "<<current_pose_.y<<" theata is "<<current_pose_.theta);
    Eigen::Vector2d direction_array(cos(current_pose_.theta),sin(current_pose_.theta));

    Eigen::Vector2d distance_array(goal_pose.x-current_pose_.x,goal_pose.y-current_pose_.y);
    double dx = std::abs(goal_pose.x-current_pose_.x);
    double dy = std::abs(goal_pose.y-current_pose_.y);
    double dtheta = std::abs(goal_pose.theta - current_pose_.theta);
    double distance = distance_array.dot(direction_array);
    return dx < 0.4 && dy <0.4 && dtheta <0.4;
    // return std::abs(distance)<0.2||distance<0;
}
void NavCore::setGoal(const geometry_msgs::Pose2D & goal2d)
{
    move_base_msgs::MoveBaseGoal goal{};
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal2d.x;
    goal.target_pose.pose.position.y = goal2d.y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw (goal2d.theta);
    moveBaseClient->sendGoal(goal);
}
void NavCore::cancelAllGoals()
{
    moveBaseClient->cancelAllGoals();
}
bool NavCore::clearCostMap()
{
        return client.call(clear_costmap_srv_);
}