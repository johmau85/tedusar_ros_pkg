/** \brief
 * Node for 3d search from the kinect where the hedadlles rgbdslam is used and the services to get 3d points
 * the node search for victims in the 3d point clouds that are received from the rgbdslam node
 *
 *  \file tedusar_path_follower.cpp
 *  \author Peter Lepej
 *  \date 12.12.2012
 *  \version 1.0
 */

#ifndef tedusar_path_follower_hpp___
#define tedusar_path_follower_hpp___ //header is included only once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <ros/callback_queue.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>

#include <tedusar_nav_msgs/ExecutePathAction.h>
#include <actionlib/server/simple_action_server.h>
#include <base_local_planner/costmap_model.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <cmath>

using namespace tedusar_nav_msgs;
using namespace std;
typedef actionlib::SimpleActionServer<ExecutePathAction> ExecutePathActionServer;

#define PI 3.14159265


namespace tedusar_path_follower  //split up code more classes whit the same name
{
    class tedusar_path_follower
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        tedusar_path_follower(const tedusar_path_follower &src);

        //costmap_2d::Costmap2DROS* costmap_2d_ros_;
        //base_local_planner::CostmapModel* costmap_model_;

        protected:
        ros::NodeHandle nh_;
        ExecutePathActionServer execute_path_action_server_;
        ros::Subscriber path_3d_sub;
        ros::Subscriber pose_update_sub;
        ros::Subscriber imu_data_sub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher path_pub;
        ros::Publisher local_path_pub;
        ros::Publisher marker_pub;

        public:
        std::string path_topic;
        std::string pose_update_topic;
        std::string cmd_vel_out;
        std::string map_link;
        std::string base_link;
        std::string imu_topic;
        nav_msgs::Odometry odom;
        geometry_msgs::Twist cmd;
        //for visualization
        nav_msgs::Path curr_path;
        nav_msgs::Path calc_path;
        nav_msgs::Path local_calc_path;

        ExecutePathResult action_result;
        ExecutePathFeedback feedback;

        bool alignment_finished;
        bool show_trajectory_planing;
        bool move_robot;
        bool enable_angle_compensation;
        bool enable_ground_compensation;
        bool enable_velocity_encrease;

        double angle_diff;
        double pub_cmd_hz;
        double max_lin_speed, min_lin_speed;
        double max_rot_speed, min_rot_speed;
        double execution_period;
        double alignment_angle;
        double roll, pitch, yaw;
        double update_skip;
        double curr_dist;
        double rot_dir_opti, rot_vel_dir;
        double lin_vel, rot_vel, dist, lin_vel_ref;
        double points[50][2];
        double max_H, Wid, rad;
        double global_goal_tolerance;
        double th_po_x, th_po_y, fi_po_x, fi_po_y, se_po_x, se_po_y;
        double dirx, diry;
        double sideA, sideB, sideC;
        double ss, area, tmp_H;
        double al_an_diff;
        double midX, midY;
        double dx, dy;
        double distt, pdist;
        double mDx, mDy;
        double old_pos_x;
        double old_pos_y;
        double glo_pos_diff_x, glo_pos_diff_y;
        double rot_correction_factor;
        double imu_roll, imu_pitch, imu_yaw;
        double lower_al_angle, upper_al_angle;

        int co_unchanged, co_points;
        int psize, st_point, path_po_lenght;
        int err_cont;
        int oscilation_rotation;

        tf::TransformListener listener;
        tf::StampedTransform transform;
        tf::Vector3 origin;
        tf::Quaternion rotation;
        tf::Vector3 axis;
        ros::Time now;

        /** \brief Standard construktor.
         *
         */
        tedusar_path_follower(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~tedusar_path_follower(); //always virtual

        //initialization function
        void init();
        //data calback functions
        void path_cb(const nav_msgs::Path::ConstPtr&  path);
        void pose_update(const nav_msgs::Odometry pose);
        void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
        //circle fiting calculation functions
        void calculate_cmd();
        void calc_local_path();
        void calculate_al_rot();
        void calc_angel_compensation();
        void velocity_increase();
        void resetPathFollowing();
        void calc_ground_compensation();
        //action server
        void executePathCB(const ExecutePathGoalConstPtr &goal);
        //colision checker
        void checkForColision();
    };
}

#endif
