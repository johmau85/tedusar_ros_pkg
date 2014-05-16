/** \brief
 *
 *  \file tedusar_path_follower.cpp
 *  \author Peter Lepej
 *  \date 12.12.2013
 *  \version 1.0
 */

#include <tedusar_daf_path_follower/tedusar_daf_path_follower.hpp>

namespace tedusar_path_follower
{
    /****************************************************************
     * Here we can set up the parameters
     */
    tedusar_path_follower::tedusar_path_follower(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
      execute_path_action_server_(nh_, "execute_path", boost::bind(&tedusar_path_follower::executePathCB, this, _1), false)//,
      //listener(),
      //costmap_2d_ros_(NULL)
    {

        //costmap_2d_ros_ = new costmap_2d::Costmap2DROS("global_costmap", listener);

        ros::NodeHandle private_nh("~");

        private_nh.param<double>("pub_cmd_hz", pub_cmd_hz, 10);
        private_nh.param<std::string>("path_topic", path_topic, string("/exploration_path"));
        private_nh.param<std::string>("sub_pose_update_topic", pose_update_topic, string("/odom"));
        private_nh.param<std::string>("imu_data", imu_topic, string("/imu_data"));
        private_nh.param<std::string>("out_cmd_vel", cmd_vel_out, string("/cmd_vel"));
        private_nh.param<std::string>("map_link", map_link, string("/map"));
        private_nh.param<std::string>("base_link", base_link, string("/base_footprint"));
        private_nh.param<double>("max_lin_speed", max_lin_speed, 0.3);
        private_nh.param<double>("min_lin_speed", min_lin_speed, 0.1);
        private_nh.param<double>("max_rot_speed", max_rot_speed, 0.5);
        private_nh.param<double>("min_rot_speed", min_rot_speed, 0.1);
        private_nh.param<double>("rot_correction_factor", rot_correction_factor, 1);
        private_nh.param<double>("execution_period", execution_period, 1.0); //0.0 execute at pose update -- is smaller have to decrease local tolerance
        private_nh.param<double>("update_skip_until_vel_increase", update_skip, 5);
        private_nh.param<double>("global_goal_tolerance", global_goal_tolerance, 0.1);
        private_nh.param<double>("lower_al_angle", lower_al_angle, 0.2);              //this angle determine when angle correction is executed
        private_nh.param<double>("upper_al_angle", upper_al_angle, 0.6);              //this angle determine middle robot correction wich is compensate in linear movment
                                                                                      //smaller correction angle means better fiting of trajectories
        private_nh.param<bool>("enable_angle_compensation", enable_angle_compensation, true);
        private_nh.param<bool>("enable_ground_compensation", enable_ground_compensation, true);
        private_nh.param<bool>("enable_velocity_encrease", enable_velocity_encrease, true);
        private_nh.param<bool>("show_trajectory_planing", show_trajectory_planing, true);
    }
    /****************************************************************
     *
     */
    tedusar_path_follower::~tedusar_path_follower()
    {
//        if(costmap_2d_ros_)
//        {
//            delete costmap_2d_ros_;
//        }
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void tedusar_path_follower::init()
    {
        //path_3d_sub = nh_.subscribe<nav_msgs::Path>(path_topic,50 , &tedusar_path_follower::path_cb, this);
        pose_update_sub = nh_.subscribe<nav_msgs::Odometry>(pose_update_topic ,10 , &tedusar_path_follower::pose_update, this);
        cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_out, 1);
        imu_data_sub = nh_.subscribe<sensor_msgs::Imu>(imu_topic, 1, &tedusar_path_follower::imuCallback, this);

        //path following visualization
        if(show_trajectory_planing)
        {
            path_pub = nh_.advertise<nav_msgs::Path>("/calc_path", 1);
            local_path_pub = nh_.advertise<nav_msgs::Path>("/local_calc_path", 1);
            marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        }

        //reset initial settings
        resetPathFollowing();

        // start  action server
        execute_path_action_server_.start();
    }
    /*****************************************************************************************************************
     * Resets Path Following
     */
    void tedusar_path_follower::resetPathFollowing()
    {
        alignment_finished = false;
        co_unchanged = 0;
        st_point = 0;
        path_po_lenght = 0;
        lin_vel = max_lin_speed/2;

        //save reference for linar velocity
        lin_vel_ref = lin_vel;

        //rot_vel_dir = 1;
        //rot_dir_opti = 1;
        imu_yaw = 0;
        imu_pitch = 0;
        imu_roll = 0;
        old_pos_x = 0;
        old_pos_y = 0;
        rad = 0;
        err_cont = 0;
        oscilation_rotation = 1;

        move_robot = false;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub.publish(cmd);
    }
    /****************************************************************
     * Imu Data Callback
     */
    void tedusar_path_follower::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
    {
        tf::Quaternion imu_quaternion;
        tf::quaternionMsgToTF(imu_msg->orientation, imu_quaternion);
        tf::Transform imu_orientation(imu_quaternion, tf::Vector3(0,0,0));
        imu_orientation.getBasis().getEulerYPR(imu_yaw, imu_pitch, imu_roll);
     }
    /*****************************************************************************************************************
     * Pose update
     */
    void tedusar_path_follower::pose_update(const nav_msgs::Odometry pose)
    {
        //check if transformation is avaliable
        if(move_robot)
        {
            //if path is received calculate path
            if(listener.canTransform(map_link, base_link, ros::Time(0)))
            {
                now = ros::Time::now();
                listener.lookupTransform(map_link, base_link, ros::Time(0), transform);   //daj ven na params
                origin = transform.getOrigin();
                rotation = transform.getRotation();
                axis = rotation.getAxis();
                odom.pose.pose.position.x = origin.x();
                odom.pose.pose.position.y = origin.y();
                odom.pose.pose.position.z = origin.z();
                tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
                tf::Matrix3x3 m(q);
                m.getRPY(roll, pitch, yaw);

                //calculate local path
                calc_local_path();

                //increase velocity if robot dose not move
                velocity_increase();
            }else
            {
                ROS_WARN("There is no transfrmationa avaliable from %s to %s", map_link.c_str(), base_link.c_str());
            }
        }
    }
    /*****************************************************************************************************************
     * Check for colision
     */
    void tedusar_path_follower::checkForColision()
    {

        /*
        //map

        //robot pose
        tf::Stamped<tf::Pose> robot_pose_tf;
        costmap_2d_ros_->getRobotPose(robot_pose_tf);
        geometry_msgs::PoseStamped start_pose;
        tf::poseStampedTFToMsg(robot_pose_tf, start_pose);
        ROS_INFO("Robot start pose tf: %f %f", start_pose.pose.position.x, start_pose.pose.position.y);

        //robot footprint
        costmap_2d::Costmap2D costmap;
        costmap_2d_ros_->getCostmapCopy(costmap);

        costmap_model_ = new base_local_planner::CostmapModel(costmap);

        std::vector<geometry_msgs::Point> oriented_footprint;


        tf::Point position_to_object(0.5, 0.5, 0); //robot position

        tf::Quaternion orientation_to_object = tf::createQuaternionFromYaw(
                                atan2(-0.5, -0.5));

        tf::Pose pose_candidate;

        pose_candidate.setRotation(orientation_to_object);
        pose_candidate.setOrigin(position_to_object);

        tf::StampedTransform transform_cost;

        ///????????????????????????????????????
        listener.lookupTransform(map_link, base_link, ros::Time(0),
                                                transform_cost);

        //transform pose candidate to map frame
        pose_candidate = transform_cost.inverseTimes(pose_candidate);


        double x = pose_candidate.getOrigin()[0];
        double y = pose_candidate.getOrigin()[1];
        double yaw = tf::getYaw(pose_candidate.getRotation());

        costmap_2d_ros_.getOrientedFootprint(x, y, yaw, oriented_footprint);
        geometry_msgs::Point point_msg_in_map;
        tf::pointTFToMsg(pose_candidate.getOrigin(), point_msg_in_map);

        double cost = costmap_model_->footprintCost(point_msg_in_map, oriented_footprint, costmap_2d_ros_.getInscribedRadius(),costmap_2d_ros_.getCircumscribedRadius());



        //check for colision

        //make action
        */

    }
    /*****************************************************************************************************************
     * Path Callback From Topic
     */
    void tedusar_path_follower::path_cb(const nav_msgs::Path::ConstPtr&  path)
    {
        curr_path = *path;
        psize = (int)curr_path.poses.size();
        dist = abs(execution_period*max_lin_speed);

        if(psize > 0) {
            move_robot = true;
        }else{
            move_robot = false;}

        ROS_INFO("New Path Received from topic. Path seq: %i size: %i distance to plan path: %f", curr_path.header.seq, psize, dist);
    }
    /*****************************************************************************************************************
     * calculate Fitter Circular Path
     */
    void tedusar_path_follower::calc_local_path()
    {
        path_po_lenght = 0;
        //calculate path_po_lenght
        for(int i=0; i < psize; i++)
        {
            double curr_dist_x = abs(odom.pose.pose.position.x - curr_path.poses[i].pose.position.x);
            double curr_dist_y = abs(odom.pose.pose.position.y - curr_path.poses[i].pose.position.y);
            double curr_dist = sqrt(curr_dist_x*curr_dist_x + curr_dist_y*curr_dist_y);

            if(abs(curr_dist) > dist) //search for points
            {
                continue;
            }
            path_po_lenght = path_po_lenght + 1;
        }

        double min_dif = 10.0;

        //start point from robot current pose
        points[0][0] = odom.pose.pose.position.x;
        points[0][1] = odom.pose.pose.position.y;

        //search for closest point to path
        for(int i=0; i < psize; i++)
        {
            double po_dist = sqrt((curr_path.poses[i].pose.position.x - points[0][0])*(curr_path.poses[i].pose.position.x - points[0][0]) + (curr_path.poses[i].pose.position.y - points[0][1])*(curr_path.poses[i].pose.position.y - points[0][1]));
            if(abs(po_dist) < min_dif)
            {
                min_dif = abs(po_dist);
                st_point = i;
            }
        }
        //ROS_INFO("Founded closest point on path: x: %f y: %f at postition: %i", curr_path.poses[st_point].pose.position.x, curr_path.poses[st_point].pose.position.x, st_point);

        //calculate execution path distance
        co_points = 0;
        for(int i=st_point; i < (st_point+path_po_lenght); i++)
        {
            if(i > (psize-2))
            {
                co_points = co_points +1;
                points[co_points][0] = curr_path.poses[i].pose.position.x;
                points[co_points][1] = curr_path.poses[i].pose.position.y;
                i = (st_point+path_po_lenght)+1;
            }else
            {
                co_points = co_points +1;
                points[co_points][0] = curr_path.poses[i].pose.position.x;
                points[co_points][1] = curr_path.poses[i].pose.position.y;
            }
        }

        th_po_x = 0; th_po_y = 0; fi_po_x = 0; fi_po_y = 0;
        se_po_x = 0; se_po_y = 0; dirx = 1; diry = -1; max_H = 0;

        //calculate triangle height height
        for(int i=0; i < co_points; i++)
        {
                                           //p1            p2              p3
            //ROS_INFO("Points X: %f %f %f", points[0][0], points[i][0], points[co_points][0]);
            //ROS_INFO("Points Y: %f %f %f", points[0][1], points[i][1], points[co_points][1]);
            sideA = sqrt(((points[0][0] - points[i][0])*(points[0][0] - points[i][0])) + (points[0][1] - points[i][1])*(points[0][1] - points[i][1]));
            sideB = sqrt(((points[i][0] - points[co_points][0])*(points[i][0] - points[co_points][0])) + (points[i][1] - points[co_points][1])*(points[i][1] - points[co_points][1]));
            sideC = sqrt(((points[co_points][0] - points[0][0])*(points[co_points][0] - points[0][0])) + (points[co_points][1] - points[0][1])*(points[co_points][1] - points[0][1]));
            //ROS_INFO("triangle sides: %f %f %f", sideA, sideB, sideC);
            ss = (sideA + sideB + sideC)/2;
            area = sqrt(ss*(ss-sideA)*(ss-sideB)*(ss-sideC));
            //determine params for radius calculation
            tmp_H = (area*2)/sideC;

            if(tmp_H > max_H)
            {
                max_H = tmp_H;
                float det_dir = (points[co_points][0] - points[1][0])*(points[i][1] - points[0][1]) - (points[co_points][1] - points[0][1])*(points[i][0]- points[0][0]);
                se_po_x = points[i][0];
                se_po_y = points[i][1];

                if(det_dir > 0)
                {
                    dirx = -1;
                    diry = 1;
                    rot_vel_dir = -1;
                }else
                {
                    dirx = 1;
                    diry = -1;
                    rot_vel_dir = 1;
                }
            }
            Wid = sideC;
        }

        //if local path is too short
        if(co_points < 3)
        {
            max_H = 0.001;
        }
        //smooth local path
        //max_H = max_H/2;

        //calculate ground compensation, which modifiy max_H and W
        calc_ground_compensation();

        fi_po_x = points[0][0];
        fi_po_y = points[0][1];
        th_po_x = points[co_points][0];
        th_po_y = points[co_points][1];

        //calculate radious
        rad = max_H/2 + (Wid*Wid)/(8*max_H);
        //ROS_INFO("Fitted circle radius: %f", rad);

        //calculating circle center
        midX = (points[0][0] + points[co_points][0])/2;
        midY = (points[0][1] + points[co_points][1])/2;
        dx = (points[0][0] - points[co_points][0])/2;
        dy = (points[0][1] - points[co_points][1])/2;
        distt = sqrt(dx*dx + dy*dy);
        pdist = sqrt(rad*rad - distt*distt);
        mDx = dirx*dy*pdist/distt;
        mDy = diry*dx*pdist/distt;

        //calculate alignemnt angle
        double curr_dist_x = points[0][0] -  (midX + mDx);
        double curr_dist_y = points[0][1] - (midY + mDy);

        //correct angle directions
        if((curr_dist_x < 0)&&(curr_dist_y < 0))
        {
            alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*PI/2;
        }
        else if((curr_dist_x > 0)&&(curr_dist_y > 0))
        {
            alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*PI/2;
        }
        else if((curr_dist_x < 0)&&(curr_dist_y > 0))
        {
            alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*PI/2;
        }
        else if((curr_dist_x > 0)&&(curr_dist_y < 0))
        {
            alignment_angle = atan2(curr_dist_y,curr_dist_x) + rot_vel_dir*PI/2;
        }

        //reduce angle on -PI to +PI
        if(alignment_angle > PI)
        {
            alignment_angle = alignment_angle - 2*PI;
        }
        if(alignment_angle < -PI)
        {
            alignment_angle = alignment_angle + 2*PI;
        }

        if(isnan(alignment_angle))
        {
            ROS_WARN("Alignment Angle can not be computed!");
        }

        //ROS_INFO("Alignment angle is: %f", alignment_angle);
        if(isnan(alignment_angle))
        {
            ROS_INFO("Alignment angle is nan - return to calc_local_path");
            calc_local_path();
        }

        //send feedback
        feedback.feedback = (int)round((100*st_point)/psize); //calculated progress relative to given path
        execute_path_action_server_.publishFeedback(feedback);

        //display of all lines to plan a path
        if(show_trajectory_planing == true)
        {
            uint32_t shape = visualization_msgs::Marker::CUBE;

            local_calc_path.header.frame_id = map_link;
            local_calc_path.poses.resize(360);

            for(int d= 0; d < 360; d++)
            {
                double xp = sin(d*0.0174532925)*rad + (midX + mDx);
                double yp = cos(d*0.0174532925)*rad + (midY + mDy);
                local_calc_path.poses[d].pose.position.x = xp;
                local_calc_path.poses[d].pose.position.y = yp;
                local_calc_path.poses[d].pose.position.z = 0;
            }
            //publish circle
            local_path_pub.publish(local_calc_path);

            //pub triangle
            calc_path.header.frame_id = map_link;
            calc_path.poses.resize(4);

            //pub path
            calc_path.poses[0].pose.position.x = points[0][0];
            calc_path.poses[0].pose.position.y = points[0][1];
            calc_path.poses[0].pose.position.z = 0;
            calc_path.poses[1].pose.position.x = se_po_x;
            calc_path.poses[1].pose.position.y = se_po_y;
            calc_path.poses[1].pose.position.z = 0;
            calc_path.poses[2].pose.position.x = points[co_points][0];
            calc_path.poses[2].pose.position.y = points[co_points][1];
            calc_path.poses[2].pose.position.z = 0;
            calc_path.poses[3].pose.position.x = points[0][0];
            calc_path.poses[3].pose.position.y = points[0][1];
            calc_path.poses[3].pose.position.z = 0;
            calc_path.poses.resize(4);
            path_pub.publish(calc_path);

            //start point
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_link;
            marker.header.stamp = ros::Time::now();

            marker.ns = "first_point";
            marker.id = 0;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = fi_po_x;
            marker.pose.position.y = fi_po_y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            // Publish the marker
            marker_pub.publish(marker);

            marker.ns = "second_point";
            marker.id = 2;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = se_po_x;
            marker.pose.position.y = se_po_y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            // Publish the marker
            marker_pub.publish(marker);

            //end point
            marker.ns = "third_point";
            marker.id = 4;
            marker.type = shape;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = th_po_x;
            marker.pose.position.y = th_po_y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            // Publish the marker
            marker_pub.publish(marker);
        }
    }
    /*****************************************************************************************************************
     * Compensate Ground Inclinations
     */
    void tedusar_path_follower::calc_ground_compensation()
    {
        if(enable_ground_compensation == true)
        {
            //ROS_INFO("Distances before ground compensation: max_H: %f and W: %f", max_H, Wid);
            double dh = abs((max_H/cos(imu_roll))-max_H);
            double dw = abs((Wid/cos(imu_pitch))-Wid);
            //ROS_INFO("Angle compensation diferences: dh: %f, dw: %f", dh, dw);
            max_H = max_H + dh;
            Wid = Wid + dw;
            //ROS_INFO("Distances after ground compensation: max_H: %f and W: %f", max_H, Wid);
        }
    }
    /*****************************************************************************************************************
     * calculate Current Alignment Rotation
     */
    void tedusar_path_follower::calculate_al_rot()
    {
        //define rotation direction
        angle_diff = alignment_angle - yaw;

        if(fabs(angle_diff) > M_PI)
        {
            if(angle_diff > 0)
            {
                 angle_diff = -2 * M_PI + fabs(angle_diff);
            }
            else
            {
                angle_diff = 2 * M_PI - fabs(angle_diff);
            }
        }

        if(angle_diff > 0){rot_dir_opti = 1;}
        else{ rot_dir_opti = -1;}
    }
    /*****************************************************************************************************************
     * Action Callback for Path from Exploration Controller
     */
    void tedusar_path_follower::executePathCB(const ExecutePathGoalConstPtr &goal)
    {
        resetPathFollowing();

        curr_path = goal->path;

        psize = (int)curr_path.poses.size();
        dist = abs(execution_period*max_lin_speed);

        if(psize > 0)
        {
            ROS_INFO("New Path Received from Action. Path seq: %i size: %i distance to plan path: %f", curr_path.header.seq, psize, dist);
            move_robot = true;

            ros::Rate r(pub_cmd_hz);
            while(ros::ok())
            {
                if(execute_path_action_server_.isPreemptRequested() || !move_robot)
                {
                    move_robot = false;
                    resetPathFollowing();
                    execute_path_action_server_.setPreempted();
                    ROS_INFO("path execution is preempted");
                    return;
                }
                calculate_cmd();
                r.sleep();
            }
        }else{
            move_robot = false;

            //sends aborted feedback
            action_result.result = 1;
            execute_path_action_server_.setAborted(action_result, std::string("Path size is 0"));
        }
    }

    /*****************************************************************************************************************
     * Compensate Current Angle Difreence
     */
    void tedusar_path_follower::calc_angel_compensation()
    {
        if(enable_angle_compensation == true)
        {
            //cosider space
            angle_diff = alignment_angle - yaw;

            if(fabs(angle_diff) > M_PI)
            {
                if(angle_diff > 0)
                {
                     angle_diff = -2 * M_PI + fabs(angle_diff);
                }
                else
                {
                    angle_diff = 2 * M_PI - fabs(angle_diff);
                }
            }

            double add_al_rot = abs(angle_diff/(execution_period));
            //ROS_INFO("Additional Alignment Rotation: %f", add_al_rot);

            rot_vel = rot_vel_dir * lin_vel/rad*rot_correction_factor + rot_dir_opti*add_al_rot;
        }else
        {
            rot_vel = rot_vel_dir * lin_vel/rad*rot_correction_factor;
        }
    }
    /*****************************************************************************************************************
     * Increase Velocity if Robot Dose not Move
     */
    void tedusar_path_follower::velocity_increase()
    {

        //check if there is no change in position it increase linear speed
        double pose_diff_x = fabs(odom.pose.pose.position.x - old_pos_x);
        double pose_diff_y = fabs(odom.pose.pose.position.y - old_pos_y);

        if((pose_diff_x <= std::numeric_limits<double>::epsilon()) && (pose_diff_y <= std::numeric_limits<double>::epsilon()))
        {
            ++co_unchanged;

            if (co_unchanged > update_skip)
            {
                co_unchanged = 0;

                if(enable_velocity_encrease == true)
                {

                    lin_vel = lin_vel + max_lin_speed/10;

                    if(lin_vel >= max_lin_speed)
                    {
                        lin_vel = max_lin_speed;

                        //goal reach reporting
                        action_result.result = 1;
                        execute_path_action_server_.setAborted(action_result, std::string("Robot cannot move, maximum speed reached!"));
                        resetPathFollowing();
                        ROS_WARN("Robot cannot move! Maximum speed reached!");
                    }
                }else
                {
                    action_result.result = 1;
                    execute_path_action_server_.setAborted(action_result, std::string("Robot cannot move!"));
                    resetPathFollowing();
                    ROS_WARN("Robot cannot move!");
                }
            }
        }
        else
        {
            //else reference linear velocity is used
            old_pos_x = odom.pose.pose.position.x;
            old_pos_y = odom.pose.pose.position.y;
            co_unchanged = 0;
            lin_vel = lin_vel_ref;
        }
    }

    /*****************************************************************************************************************
     * calculate and Publish Velocity Commands
     */
    void tedusar_path_follower::calculate_cmd()
    {
        if(move_robot == true)
        {
            ROS_INFO_ONCE("Start calculating cmd velocity!");
            //do algnment if angle is too large do alignment
            al_an_diff = alignment_angle - yaw;

            if(al_an_diff > M_PI)
            {
                al_an_diff = 2 * M_PI - al_an_diff;
            }

            calculate_al_rot();

             // if difference is larger thatn both tresholds angle_correction and middle al_offset
            if((fabs(al_an_diff) > (lower_al_angle + upper_al_angle))||(alignment_finished == false))
            {
                // turn in place
                //ROS_INFO("ROBOT IS ALIGNING || yaw: %f angle: %f", yaw*57.2957795, alignment_angle*57.2957795);
                if (yaw > alignment_angle)
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = rot_dir_opti*max_rot_speed/2;
                }
                else if (yaw < alignment_angle)
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = rot_dir_opti*max_rot_speed/2;
                }

                if(fabs(al_an_diff) < lower_al_angle/2)
                {
                    alignment_finished = true;
                    ROS_INFO("Alignment completed!");
                    err_cont = 0;
                }else
                {
                    alignment_finished = false;

                    //if robot misses alignment angle
                    ++err_cont;
                    if(err_cont > pub_cmd_hz*8) //second for delay
                    {
                       action_result.result = 1;
                       execute_path_action_server_.setAborted(action_result, std::string("Robot cannot move, maximum speed reached!"));
                       resetPathFollowing();
                       ROS_WARN("If your robot is oscilating, please increase lower_al_angle and upper_al_angle parameter (recommended: lower_al_angle=0.6, upper_al_angle=1.0)");
                       ROS_WARN("Requesting new path!");

                    }
                }
            }
            // else if difference is between lower treshold (angle correction) and upper treshold (middle_al_offset) the angle compensation is used
            else if((fabs(al_an_diff) > lower_al_angle)&&((fabs(al_an_diff) < (lower_al_angle + upper_al_angle))))  //||(alignment_finished == false)
            {
                //ROS_INFO("DRIVE ROBOT MIDDLE STAGE || yaw: %f al_angle: %f", yaw*57.2957795, alignment_angle*57.2957795);

                calculate_al_rot();

                //add additional rotation speed based on ground
                calc_angel_compensation();

                cmd.linear.x = lin_vel;
                cmd.angular.z = rot_vel;

                //check for global goal proximitiy
                glo_pos_diff_x = abs(odom.pose.pose.position.x - curr_path.poses[psize-1].pose.position.x);
                glo_pos_diff_y = abs(odom.pose.pose.position.y - curr_path.poses[psize-1].pose.position.y);
                if((glo_pos_diff_x < global_goal_tolerance)&&(glo_pos_diff_y < global_goal_tolerance))
                {
                    ROS_INFO("GLOBAL GOAL REACHED");
                    cmd.linear.x = 0;
                    cmd.angular.z = 0;
                    //goal reach reporting
                    action_result.result = 0;
                    execute_path_action_server_.setSucceeded(action_result);
                    move_robot = false;
                }
            }
            //if difference is below lower treshold ()
            else
            {
                //ROS_INFO("DRIVE ROBOT || yaw: %f al_angle: %f", yaw*57.2957795, alignment_angle*57.2957795);

                calculate_al_rot();

                rot_vel = rot_vel_dir * lin_vel/rad*rot_correction_factor;  //pazi za meso je dva krat

                cmd.linear.x = lin_vel;
                cmd.angular.z = rot_vel;

                //check for global goal proximitiy
                glo_pos_diff_x = fabs(odom.pose.pose.position.x - curr_path.poses[psize-1].pose.position.x);
                glo_pos_diff_y = fabs(odom.pose.pose.position.y - curr_path.poses[psize-1].pose.position.y);
                if((glo_pos_diff_x < global_goal_tolerance)&&(glo_pos_diff_y < global_goal_tolerance))
                {
                    ROS_INFO("GLOBAL GOAL REACHED");
                    cmd.linear.x = 0;
                    cmd.angular.z = 0;
                    //goal reach reporting
                    action_result.result = 0;
                    execute_path_action_server_.setSucceeded(action_result);
                    move_robot = false;

                }
             }

            //publish commad
            cmd_vel_pub.publish(cmd);
            //ROS_INFO("published velocity: lin: %f rot: %f", cmd.linear.x, cmd.angular.z);
        }
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "tedusar_path_follower");
    ros::NodeHandle nh;

    //for registering the node
    tedusar_path_follower::tedusar_path_follower tedusar_path_follower_handler(nh);

    tedusar_path_follower_handler.init();                     //initize

    ros::spin();
    return 0;
}
