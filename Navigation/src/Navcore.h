#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "std_srvs/Empty.h"
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <Eigen/Core>
#include <fstream>
struct targetPose {
    geometry_msgs::Pose2D pose;
    enum TargetAction {
      shopping,
      cooking,
      serving,
    } targetAction{};
};

class NavCore {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
public:


    enum MoveBaseActionResult {
        PENDDING,
        ACTIVE,
        PREEMPTED,
        SUCCEEDED,
        ABORTED,
        REJECTED,
        PREEMPTING,
        RECALLING,
        RECALLED,
        LOST,
        EMPTY,
    };
    
private:
    const std::string BASE_FOOT_PRINT_;
    const std::string MAP_FRAME_;
    const std::string FILE_NAME_;
    mutable boost::shared_mutex action_result_mutex_{};

    ros::NodeHandle nh;
    ros::Subscriber action_result_sub;
    MoveBaseClient *moveBaseClient;
    

    std_srvs::Empty clear_costmap_srv_;
    ros::ServiceClient client;

    bool isMoveBaseClientConnected_{};
    geometry_msgs::Pose2D current_pose_{};
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;
    void actionResultCallback(const move_base_msgs::MoveBaseActionResult &msg);
    // void actionResultCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
public:
  MoveBaseActionResult moveBaseActionResult_{EMPTY};
    NavCore(std::string base_foot_print, std::string map_frame);

    ~NavCore();
    void saveGoal(targetPose& pose);

    bool isMoveBaseClientConnected()const {return isMoveBaseClientConnected_;};
    bool clearCostMap();
    void cancelAllGoals();
    bool getCurrentPose_bool(geometry_msgs::Pose2D & current_pose);
    void setGoal(const geometry_msgs::Pose2D &goal2d);

    MoveBaseActionResult getMoveBaseActionResult()
    {
        boost::unique_lock<boost::shared_mutex> writeLock(action_result_mutex_);
        MoveBaseActionResult temp{moveBaseActionResult_};
        moveBaseActionResult_=MoveBaseActionResult::EMPTY;
        return temp;
    };

    const geometry_msgs::Pose2D &getCurrentPose(const std::string &target_frame, const std::string &source_frame);

    bool isGoalPassed(const geometry_msgs::Pose2D &goal_pose);
};