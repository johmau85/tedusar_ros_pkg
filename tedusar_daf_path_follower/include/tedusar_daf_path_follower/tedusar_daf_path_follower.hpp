/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Peter Lepej,
 *                      Faculty of Electrical Engineerign anc Computer Scienece,
 *                      University of Maribor
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef tedusar_path_follower_hpp___
#define tedusar_path_follower_hpp___

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tedusar_nav_msgs/ExecutePathAction.h>
#include <actionlib/server/simple_action_server.h>

#define PI 3.14159265

using namespace tedusar_nav_msgs;
using namespace std;
typedef actionlib::SimpleActionServer<ExecutePathAction> ExecutePathActionServer;

namespace tedusar_path_follower
{
    class tedusar_path_follower
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        tedusar_path_follower(const tedusar_path_follower &src);

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

	//actions
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
	double stability_angle;

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
        //geometry calculation calback functions
        void path_cb(const nav_msgs::Path::ConstPtr&  path);
        void pose_update(const nav_msgs::Odometry pose);
        void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);
        void calculate_cmd();
        void calc_local_path();
        void calculate_al_rot();
        void calc_angel_compensation();
        void velocity_increase();
        void resetPathFollowing();
        void calc_ground_compensation();
        //action server
        void executePathCB(const ExecutePathGoalConstPtr &goal);
	void check_robot_stability();
    };
}

#endif
