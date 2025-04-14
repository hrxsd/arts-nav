#! /usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from costmap_converter.msg import ObstacleArrayMsg
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import tf
from collections import namedtuple
from RDA_planner.mpc import MPC
from math import atan2
from gctl.curve_generator import curve_generator
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
import cv2
from math import cos, sin

# 导入 actionlib 和 move_base Action 消息
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from actionlib_msgs.msg import GoalStatus # 用于获取状态信息

robot_tuple = namedtuple(
    "robot_tuple", "G h cone_type wheelbase max_speed max_acce dynamics"
)
rda_obs_tuple = namedtuple(
    "rda_obs_tuple", "center radius vertex cone_type velocity"
)  # vertex: 2*number of vertex


class rda_core_1:
    def __init__(self) -> None:

        # publish topics
        self.vel_pub = rospy.Publisher("/rda_cmd_vel", Twist, queue_size=10)
        self.rda_path_pub = rospy.Publisher("/rda_opt_path", Path, queue_size=10)

        self.ref_path_pub = rospy.Publisher("/rda_ref_path", Path, queue_size=10)
        self.ref_states_pub = rospy.Publisher("/rda_ref_states", Path, queue_size=10)
        self.obs_pub = rospy.Publisher("/rda_obs_markers", MarkerArray, queue_size=10)
        # self.move_base_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        # 添加目标状态相关变量
        self.current_goal_pose = None  # 存储目标 PoseStamped
        self.goal_reached_by_rda = False # RDA判断是否到达目标
        # self.goal_published = False    # 不再需要，用 action client 状态代替
        self.new_goal_received = False # 是否收到新目标
        self.move_base_goal_active = False # move_base action 是否正在执行

        rospy.init_node("rda_node", anonymous=True)

        # ros parameters

        ## robot info
        robot_info = rospy.get_param(
            "~robot_info",
            {
                "vertices": None,
                "radius": None,
                "max_speed": [10, 1],
                "max_acce": [10, 0.5],
                "length": 2,
                "width": 1,
                "wheelbase": 1.5,
                "dynamics": "diff",
                "cone_type": "Rpositive",
            },
        )

        ## For rda MPC
        receding = rospy.get_param("~receding", 10)
        iter_num = rospy.get_param("~iter_num", 2)
        enable_reverse = rospy.get_param("~enable_reverse", False)
        sample_time = rospy.get_param("~sample_time", 0.1)
        process_num = rospy.get_param("~process_num", 4)
        accelerated = rospy.get_param("~accelerated", True)
        time_print = rospy.get_param("~time_print", False)
        obstacle_order = rospy.get_param("~obstacle_order", True)
        self.max_edge_num = rospy.get_param("~max_edge_num", 5)
        self.max_obstacle_num = rospy.get_param("~max_obs_num", 5)
        self.goal_index_threshold = rospy.get_param("~goal_index_threshold", 1)

        ## Tune parameters
        iter_threshold = rospy.get_param("~iter_threshold", 0.2)
        slack_gain = rospy.get_param("~slack_gain", 8)
        max_sd = rospy.get_param("~max_sd", 1.0)
        min_sd = rospy.get_param("~min_sd", 0.1)
        ws = rospy.get_param("~ws", 1.0)
        wu = rospy.get_param("~wu", 0.5)
        ro1 = rospy.get_param("~ro1", 200)
        ro2 = rospy.get_param("~ro2", 1.0)

        # reference speed
        self.ref_speed = rospy.get_param("~ref_speed", 4.0)  # ref speed

        ## for scan
        use_scan_obstacle = rospy.get_param("~use_scan_obstacle", False)
        self.scan_eps = rospy.get_param("~scan_eps", 0.2)
        self.scan_min_samples = rospy.get_param("~scan_min_samples", 6)

        ## for reference paths
        self.waypoints = rospy.get_param("~waypoints", [])
        self.loop = rospy.get_param("~loop", False)
        self.curve_type = rospy.get_param("~curve_type", "dubins")
        self.step_size = rospy.get_param("~step_size", 0.1)
        self.min_radius = rospy.get_param("~min_radius", 1.0)

        ## for frame
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.lidar_frame = rospy.get_param("~lidar_frame", "lidar_link")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        # for visualization
        self.marker_x = rospy.get_param("~marker_x", 0.05)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.1)

        # initialize
        self.robot_state = None
        self.obstacle_list = []
        self.cg = curve_generator()
        self.listener = tf.TransformListener()
        # self.ref_path_list = (
        #     self.generate_ref_path_list()
        # )  # generate the initial reference path
        self.ref_path_list = [] # 初始为空，等待 move_base 提供

        robot_info_tuple = self.generate_robot_tuple(robot_info)

        self.rda_opt = MPC(
            robot_info_tuple,
            self.ref_path_list,
            receding,
            sample_time,
            iter_num,
            enable_reverse,
            False,
            obstacle_order,
            self.max_edge_num,
            self.max_obstacle_num,
            process_num,
            accelerated,
            time_print,
            self.goal_index_threshold,
            iter_threshold=iter_threshold,
            slack_gain=slack_gain,
            max_sd=max_sd,
            min_sd=min_sd,
            ws=ws,
            wu=wu,
            ro1=ro1,
            ro2=ro2,
        )

        # --- 创建 move_base Action Client ---
        rospy.loginfo("Connecting to move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # 可以设置等待超时时间，例如 rospy.Duration(10)
        if not self.move_base_client.wait_for_server(rospy.Duration(60)):
             rospy.logerr("Could not connect to 'move_base' action server after 60 seconds!")
             # 在这里可以考虑退出或者采取其他错误处理措施
             return # 或者 rospy.signal_shutdown("...")
        rospy.loginfo("Connected to move_base action server.")
        # --- Action Client 创建完毕 ---

        # --- 订阅器部分 ---
        rospy.Subscriber("/rda_goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/rda_sub_path", Path, self.path_callback)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.global_path_callback)
        
        self.listener = tf.TransformListener()

        # Topic Subscribe
        if not use_scan_obstacle:
            rospy.Subscriber("/rda_obstacles", ObstacleArrayMsg, self.obstacle_callback)
        else:
            rospy.Subscriber("/scan", LaserScan, self.scan_callback)
    
    # --- Action Client 的回调函数 ---
    def move_base_active_cb(self):
        """move_base 开始处理目标时的回调"""
        self.move_base_goal_active = True
        rospy.loginfo("move_base action goal is active.")

    def move_base_feedback_cb(self, feedback: MoveBaseFeedback):
        """move_base 处理过程中的反馈回调 (可选)"""
        # feedback.base_position 包含 move_base 认为的当前机器人位置
        # 可以用来做一些额外的逻辑，但在这个场景下可能不是必须的
        # rospy.loginfo_throttle(5, f"move_base feedback pose: {feedback.base_position.pose.position.x}, {feedback.base_position.pose.position.y}")
        pass

    def move_base_done_cb(self, status, result: MoveBaseResult):
        """move_base 完成处理目标时的回调 (无论成功、失败还是被取消)"""
        self.move_base_goal_active = False
        status_text = GoalStatus.to_string(status)
        rospy.loginfo(f"move_base action finished with status: {status_text}")

        # 如果 move_base 成功了 (虽然我们通常会先取消它)
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("move_base reported SUCCEEDED.")
            self.goal_reached_by_rda = True # 也标记为到达
            # 这里可以添加成功后的额外逻辑

        # 如果目标被抢占 (例如，收到了新的目标)
        elif status == GoalStatus.PREEMPTED:
            rospy.logwarn("move_base goal was preempted.")
            # 如果是因为我们主动取消的 (rda到达目标)，则不需要额外操作
            # 如果是外部原因抢占，可能需要重置 rda 的状态

        # 如果目标被中止 (move_base 失败)
        elif status == GoalStatus.ABORTED:
            rospy.logerr("move_base failed to reach the goal (ABORTED).")
            # 这里可能需要停止机器人或执行错误处理
            self.goal_reached_by_rda = False # 标记未到达

        # 其他状态... REJECTED, RECALLED, LOST
        else:
             rospy.logwarn(f"move_base finished with unhandled status: {status_text}")

        # 不论 move_base 的结果如何，如果 RDA 之前判断到达了，
        # 并且我们取消了目标，我们应该保持我们的状态
        # 主要目的是通过 done_cb 重置 self.move_base_goal_active 标志
    
    # --- 控制循环 ---
    def control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.read_robot_state()
            if self.robot_state is None:
                rospy.loginfo_throttle(1, "waiting for robot states")
                rate.sleep()
                continue

            # 处理新目标 - 发送给 move_base Action Server
            # 只有在收到新目标且当前没有活动目标时才发送
            if self.new_goal_received and not self.move_base_goal_active:
                if self.current_goal_pose:
                    rospy.loginfo("Received new goal, sending to move_base action server.")
                    mb_goal = MoveBaseGoal()
                    # mb_goal.target_pose = self.current_goal_pose # 直接用 PoseStamped
                    # 或者手动填充
                    mb_goal.target_pose.header.frame_id = self.current_goal_pose.header.frame_id
                    mb_goal.target_pose.header.stamp = rospy.Time.now() # 使用当前时间戳
                    mb_goal.target_pose.pose = self.current_goal_pose.pose

                    # 发送目标，并指定回调函数
                    self.move_base_client.send_goal(mb_goal,
                                                   done_cb=self.move_base_done_cb,
                                                   active_cb=self.move_base_active_cb,
                                                   feedback_cb=self.move_base_feedback_cb)
                    self.new_goal_received = False # 标记新目标已处理
                    self.goal_reached_by_rda = False # 重置 RDA 到达状态
                    # self.move_base_goal_active 标志将由 active_cb 设置
                else:
                    rospy.logwarn("New goal received but current_goal_pose is None.")
                    self.new_goal_received = False # 避免重复警告

            # --- 主要控制逻辑 ---
            opt_vel = np.zeros((2, 1)) # 默认速度为0
            current_info = None        # 用于存储MPC的info

            # 只有当 move_base 目标处于活动状态，并且 RDA 未判断到达时，才执行 MPC
            if self.move_base_goal_active and not self.goal_reached_by_rda:
                if len(self.obstacle_list) > 0:
                    rda_obs_markers = self.convert_to_markers(self.obstacle_list)
                    self.obs_pub.publish(rda_obs_markers)

                if self.rda_opt.no_ref_path() or len(self.ref_path_list) == 0:
                    rospy.loginfo_throttle(1, "move_base active, but waiting for reference path.")
                    # opt_vel 保持为 0
                else:
                    # 发布参考路径可视化
                    ref_path = self.convert_to_path(self.ref_path_list)
                    self.ref_path_pub.publish(ref_path)

                    # 调用MPC计算
                    rospy.loginfo_throttle(1, "Performing path tracking / MPC calculation")
                    if self.max_obstacle_num == 0:
                        opt_vel, current_info = self.rda_opt.control(self.robot_state, self.ref_speed, [])
                    else:
                        opt_vel, current_info = self.rda_opt.control(self.robot_state, self.ref_speed, self.obstacle_list)

                    # 检查是否到达 (根据RDA的判断)
                    if current_info and current_info["arrive"] and not self.goal_reached_by_rda:
                        rospy.loginfo("RDA judged goal as reached.")
                        self.goal_reached_by_rda = True
                        opt_vel = np.zeros((2, 1)) # 到达时立即设置零速度

                        # --- 关键: 取消 move_base 的目标 ---
                        rospy.loginfo("Cancelling current move_base action goal.")
                        # self.move_base_client.cancel_goal() # 取消当前目标
                        self.move_base_client.cancel_all_goals() # 或者取消所有目标，更保险
                        # 注意：取消后，move_base_done_cb 会被调用，状态通常是 PREEMPTED
                        # ------------------------------------

                        # 可以在这里添加额外的“到达”逻辑，比如播放声音等
                        print("RDA: arrive at the goal!")

            # 如果 RDA 判断已到达，或者 move_base 目标不活动，则强制发送零速度
            elif self.goal_reached_by_rda or not self.move_base_goal_active:
                 if self.goal_reached_by_rda:
                      rospy.loginfo_throttle(5, "RDA goal reached state, publishing zero velocity.")
                 elif not self.move_base_goal_active:
                      rospy.loginfo_throttle(5, "No active move_base goal, publishing zero velocity.")
                 opt_vel = np.zeros((2, 1))


            # 将计算出的（或强制设置的）速度转换为Twist并发布
            vel = self.convert_to_twist(opt_vel)
            self.vel_pub.publish(vel)

            # 发布路径可视化 (只在活动且未到达时发布MPC计算的路径)
            if self.move_base_goal_active and not self.goal_reached_by_rda and current_info:
                 rda_opt_path = self.convert_to_path(current_info["opt_state_list"])
                 ref_states = self.convert_to_path(current_info["ref_traj_list"])
                 self.ref_states_pub.publish(ref_states)
                 self.rda_path_pub.publish(rda_opt_path)

            rate.sleep()

    def read_robot_state(self):

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.target_frame, self.base_frame, rospy.Time(0)
            )
            # print(trans, rot)
            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]
            self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.loginfo_throttle(
                1,
                "waiting for tf for the transform from {} to {}".format(
                    self.base_frame, self.target_frame
                ),
            )

    def obstacle_callback(self, obstacle_array):

        temp_obs_list = []

        if self.max_obstacle_num == 0:
            rospy.loginfo_once(1, "No obstacles are considered")
            return

        for obstacles in obstacle_array.obstacles:

            vertex = obstacles.polygon.points
            vertex_num = len(vertex)

            if vertex_num == 1:
                # circle obstacle

                center = np.array([[vertex[0].x], [vertex[0].y]])
                radius = obstacles.radius

                linear_x, linear_y = (
                    obstacles.velocities.twist.linear.x,
                    obstacles.velocities.twist.linear.y,
                )
                velocity = np.array([[linear_x], [linear_y]])

                circle_obs = rda_obs_tuple(center, radius, None, "norm2", velocity)

                temp_obs_list.append(circle_obs)

            elif vertex_num == 2:
                # line obstacle
                continue

            elif vertex_num > 2:
                # polygon obstacle
                vertex_list = [np.array([[p.x], [p.y]]) for p in vertex]
                vertexes = np.hstack(vertex_list)

                linear_x, linear_y = (
                    obstacles.velocities.twist.linear.x,
                    obstacles.velocities.twist.linear.y,
                )
                velocity = np.array([[linear_x], [linear_y]])

                polygon_obs = rda_obs_tuple(None, None, vertexes, "Rpositive", velocity)

                temp_obs_list.append(polygon_obs)

        self.obstacle_list[:] = temp_obs_list[:]

    # --- global_path_callback ---
    def global_path_callback(self, path: Path):
        """处理来自 move_base 的全局路径"""
        # 检查收到的路径是否为空或无效
        if not path.poses:
             rospy.logwarn_throttle(5, "Received an empty global path from move_base.")
             # 可以在这里决定是否要取消当前目标，如果路径突然消失可能意味着有问题
             # self.move_base_client.cancel_all_goals()
             return

        # 转换 Path 消息为 ref_path_list 格式
        self.ref_path_list = []
        for p in path.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            theta = self.quat_to_yaw(p.pose.orientation)
            points = np.array([x, y, theta]).reshape(3, 1)
            self.ref_path_list.append(points)

        if len(self.ref_path_list) == 0:
            rospy.logwarn_throttle(1, "Converted global path is empty, waiting for new path.")
            return

        rospy.loginfo("Reference path updated from global planner.")
        self.rda_opt.update_ref_path(self.ref_path_list)

        # 当收到新路径时，意味着 move_base 正在规划或重新规划
        # 此时应该认为目标尚未被 RDA 到达
        if self.goal_reached_by_rda:
             rospy.loginfo("Received new global path from move_base, resetting RDA goal reached state.")
             self.goal_reached_by_rda = False

        # self.goal_published 不再需要
        # self.move_base_goal_active 由 Action Client 的回调函数管理

    def path_callback(self, path):

        self.ref_path_list = []

        for p in path.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            theta = self.quat_to_yaw(p.pose.orientation)

            points = np.array([x, y, theta]).reshape(3, 1)
            self.ref_path_list.append(points)

        if len(self.ref_path_list) == 0:
            rospy.loginfo_throttle(
                1,
                "No waypoints are converted to reference path, waiting for new waypoints",
            )
            return

        rospy.loginfo_throttle(0.1, "reference path update")
        self.rda_opt.update_ref_path(self.ref_path_list)
    
    # --- goal_callback ---
    def goal_callback(self, goal: PoseStamped):
        """处理来自 /rda_goal 的目标消息"""
        rospy.loginfo(f"Received new external goal via /rda_goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")

        # 如果当前有活动目标，可以选择是取消当前目标还是忽略新目标
        if self.move_base_goal_active:
            rospy.logwarn("Received a new goal while another goal is active. Cancelling the previous goal.")
            self.move_base_client.cancel_all_goals() # 取消当前活动目标
            # 可能需要稍等一下，让取消操作完成，或者依赖 done_cb 来重置状态
            # 简单起见，我们先直接设置状态，让下一个循环发送新目标
            self.move_base_goal_active = False

        self.current_goal_pose = goal # 存储新的目标 PoseStamped
        self.new_goal_received = True   # 标记收到新目标，让 control 循环发送
        self.goal_reached_by_rda = False # 重置 RDA 到达状态
        

    def scan_callback(self, scan_data):

        ranges = np.array(scan_data.ranges)
        angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))

        point_list = []

        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (scan_data.range_max - 0.01):
                point = np.array(
                    [[scan_range * np.cos(angle)], [scan_range * np.sin(angle)]]
                )
                point_list.append(point)

        if len(point_list) < 3 or self.robot_state is None:
            rospy.loginfo_throttle(1, "No obstacles are converted to polygon")
            return

        else:

            # get the transform from lidar to target frame
            try:
                trans, rot = self.listener.lookupTransform(
                    self.target_frame, self.lidar_frame, rospy.Time(0)
                )

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]

                l_trans, l_R = self.get_transform(np.array([x, y, yaw]).reshape(3, 1))

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo_throttle(
                    1,
                    "waiting for tf for the transform from {} to {}".format(
                        self.lidar_frame, self.target_frame
                    ),
                )
                return

            # convert the points to convex polygon
            point_array = np.hstack(point_list).T
            labels = DBSCAN(
                eps=self.scan_eps, min_samples=self.scan_min_samples
            ).fit_predict(point_array)

            self.obstacle_list = []

            for label in np.unique(labels):
                if label == -1:
                    continue
                else:
                    point_array2 = point_array[labels == label]
                    rect = cv2.minAreaRect(point_array2.astype(np.float32))
                    box = cv2.boxPoints(rect)

                    vertices = box.T

                    global_vertices = l_R @ vertices + l_trans

                    self.obstacle_list.append(
                        rda_obs_tuple(None, None, global_vertices, "Rpositive", 0)
                    )

    def generate_ref_path_list(self):

        # if len(self.waypoints) == 0:
        #     return []

        # else:
        #     point_list = [
        #         np.array([[p[0]], [p[1]], [p[2]]]).astype("float64")
        #         for p in self.waypoints
        #     ]
        #     ref_path_list = self.cg.generate_curve(
        #         self.curve_type, point_list, self.step_size, self.min_radius
        #     )

        #     return ref_path_list
        # 无论 waypoints 是否存在，都返回空列表
        return []

    def convert_to_markers(self, obs_list):

        marker_array = MarkerArray()

        # obs: center, radius, vertex, cone_type, velocity

        for obs_index, obs in enumerate(obs_list):
            marker = Marker()
            marker.header.frame_id = self.target_frame

            marker.header.stamp = rospy.get_rostime()

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            marker.lifetime = rospy.Duration(self.marker_lifetime)

            # breakpoint()

            if obs.vertex is not None:

                marker.type = marker.LINE_LIST

                marker.scale.x = self.marker_x

                temp_matrix = np.hstack((obs.vertex, obs.vertex[:, 0:1]))
                for i in range(temp_matrix.shape[1] - 1):
                    vp = temp_matrix[:, i]
                    vp1 = temp_matrix[:, i + 1]

                    marker.points.append(Point(vp[0], vp[1], 0))
                    marker.points.append(Point(vp1[0], vp1[1], 0))

                marker.id = obs_index
                marker_array.markers.append(marker)

            else:
                marker.type = marker.CYLINDER

                center = obs.center
                marker.scale.x = obs.radius * 2
                marker.scale.y = obs.radius * 2
                marker.scale.z = 0.2

                marker.pose.position.x = center[0, 0]
                marker.pose.position.y = center[1, 0]

                marker.id = obs_index

                marker_array.markers.append(marker)

        return marker_array

    def convert_to_path(self, state_list):
        # from state list to path
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = self.target_frame

        for i in range(len(state_list)):
            ps = PoseStamped()

            ps.header.seq = i
            ps.header.stamp = rospy.get_rostime()
            ps.header.frame_id = self.target_frame

            ps.pose.position.x = state_list[i][0, 0]
            ps.pose.position.y = state_list[i][1, 0]
            ps.pose.orientation.w = 1

            path.poses.append(ps)

        return path

    def convert_to_twist(self, rda_vel):
        # from 2*1 vector to twist

        vel = Twist()
        vel.linear.x = rda_vel[0, 0]  # linear
        vel.angular.z = rda_vel[1, 0]  # steering

        return vel

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    @staticmethod
    def quat_to_yaw_list(quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw

    def generate_robot_tuple(self, robot_info):

        if robot_info is None:
            print("Lack of car information, please check the robot_info in config file")
            return

        if (
            robot_info.get("vertices", None) is None
            or robot_info.get("vertices", None) == "None"
        ):
            length = robot_info["length"]
            width = robot_info["width"]
            wheelbase = robot_info["wheelbase"]

            start_x = -(length - wheelbase) / 2
            start_y = -width / 2

            point0 = np.array([[start_x], [start_y]])  # left bottom point
            point1 = np.array([[start_x + length], [start_y]])
            point2 = np.array([[start_x + length], [start_y + width]])
            point3 = np.array([[start_x], [start_y + width]])

            vertex = np.hstack((point0, point1, point2, point3))

            G, h = self.generate_Gh(vertex)
        else:
            G, h = self.generate_Gh(robot_info["vertices"])

        cone_type = robot_info["cone_type"]
        max_speed = robot_info["max_speed"]
        max_acce = robot_info["max_acce"]
        dynamics = robot_info["dynamics"]

        robot_info_tuple = robot_tuple(
            G, h, cone_type, wheelbase, max_speed, max_acce, dynamics
        )

        return robot_info_tuple

    def generate_Gh(self, vertex):
        """
        vertex: 2*num
        """

        num = vertex.shape[1]

        G = np.zeros((num, 2))
        h = np.zeros((num, 1))

        for i in range(num):
            if i + 1 < num:
                pre_point = vertex[:, i]
                next_point = vertex[:, i + 1]
            else:
                pre_point = vertex[:, i]
                next_point = vertex[:, 0]

            diff = next_point - pre_point

            a = diff[1]
            b = -diff[0]
            c = a * pre_point[0] + b * pre_point[1]

            G[i, 0] = a
            G[i, 1] = b
            h[i, 0] = c

        return G, h

    def get_transform(self, state):
        """
        Get rotation and translation matrices from state.

        Args:
            state (np.array): State [x, y, theta] (3x1) or [x, y] (2x1).

        Returns:
            tuple: Translation vector and rotation matrix.
        """

        if state.shape == (2, 1):
            rot = np.array([[1, 0], [0, 1]])
            trans = state[0:2]
        else:
            rot = np.array(
                [
                    [cos(state[2, 0]), -sin(state[2, 0])],
                    [sin(state[2, 0]), cos(state[2, 0])],
                ]
            )
            trans = state[0:2]
        return trans, rot
