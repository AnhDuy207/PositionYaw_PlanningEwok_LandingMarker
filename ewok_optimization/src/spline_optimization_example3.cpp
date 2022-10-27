#include <ros/ros.h>
#include <ewok/ed_ring_buffer.h>
#include <thread>
#include <chrono>
#include <map>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>


#include <fstream>
#include <iostream>

#include <geometry_msgs/Twist.h>

#include <vector>

#include <ewok/uniform_bspline_3d_optimization.h>
#include <ewok/polynomial_3d_optimization.h>

// congtranv
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Bool.h>

const int POW = 6;       

bool initialized = false;

std::ofstream f_time, opt_time;


ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;

ros::Publisher occ_marker_pub, updated_marker_pub, free_marker_pub, dist_marker_pub, trajectory_pub, upt_marker_pub, current_traj_pub, command_pt_pub, command_pt_viz_pub;

tf::TransformListener * listener;

geometry_msgs::Point last_ctrl_point;
int target_num;
double target_error_;
std::vector<double> x_target;
std::vector<double> y_target; 
std::vector<double> z_target;
geometry_msgs::PoseStamped current_pose;
bool start_reached = false;
std_msgs::Float32MultiArray target_array;
std_msgs::Bool check_last_opt_point;

void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}
geometry_msgs::PoseStamped targetTransfer(double x, double y, double z)
{
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    return target;
}
bool checkPosition(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;
	double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;

	if(((xt - error) < xc) && (xc < (xt + error)) 
	&& ((yt - error) < yc) && (yc < (yt + error))
	&& ((zt - error) < zc) && (zc < (zt + error)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

//Depth Image Processing
void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // const float fx = 554.254691191187;
    // const float fy = 554.254691191187;
    // const float cx = 320.5;
    // const float cy = 240.5;

    // const float fx = 240; 
    // const float fy = 240;
    // const float cx = 360;
    // const float cy = 240;

    // DuyNguyen info /camera/depth/camera_info  =>K: [347.99755859375, 0.0, 320.0, 0.0, 347.99755859375, 240.0, 0.0, 0.0, 1.0]   =>>>>> fx cx fy cy
    // const float fx = 347.99755859375; 
    // const float fy = 347.99755859375;
    // const float cx = 320.0;
    // const float cy = 240.0;

    // rostopic echo /camera/depth/duy/camera_info => K: [391.49725341796875, 0.0, 360.0, 0.0, 391.49725341796875, 240.0, 0.0, 0.0, 1.0]
    const float fx = 391.49725341796875; 
    const float fy = 391.49725341796875;
    const float cx = 360.0;
    const float cy = 240.0;

    
    //transform /map & /baselink
    static tf::TransformBroadcaster br;
    tf::Transform br_transform;
    br_transform.setOrigin(tf::Vector3(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z));
    br_transform.setRotation(tf::Quaternion(current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w));
    br.sendTransform(tf::StampedTransform(br_transform, ros::Time::now(),"/map","/base_link")); 

    tf::StampedTransform transform;
    try{
        listener->lookupTransform("/map", "/camera_link", msg->header.stamp, transform);  //camera_link  camera_depth_optical_frame
    }
    catch (tf::TransformException &ex) {
        //ROS_INFO("Couldn't get transform");
        //ROS_WARN("%s",ex.what());
        return;
    }

    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    float * data = (float *) cv_ptr->image.data;

    auto t1 = std::chrono::high_resolution_clock::now();

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud;

    for(int u=0; u < cv_ptr->image.cols; u+=4) {
        for(int v=0; v < cv_ptr->image.rows; v+=4) {
            float val = data[v*cv_ptr->image.cols + u]; 

            //ROS_INFO_STREAM(val);

            if(std::isfinite(val)) {
                Eigen::Vector4f p;
                p[0] = val*(u - cx)/fx;
                p[1] = val*(v - cy)/fy;
                p[2] = val;
                p[3] = 1;
                
                p = T_w_c * p;

                cloud.push_back(p);
            }
        }
    }

    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    auto t2 = std::chrono::high_resolution_clock::now();

    if(!initialized) {
        Eigen::Vector3i idx;

        edrb->getIdx(origin, idx);

        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

        edrb->setOffset(idx);

        initialized = true;
    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);
 
        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            //ROS_INFO("Moving Volume");
            edrb->moveVolume(diff);

            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }
    }

    auto t3 = std::chrono::high_resolution_clock::now();

    edrb->insertPointCloud(cloud, origin);

    edrb->updateDistance();

    auto t4 = std::chrono::high_resolution_clock::now();

    f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
              std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

    visualization_msgs::Marker m_occ, m_free, m_dist;
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);
    edrb->getMarkerDistance(m_dist, 0.5);

    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
    dist_marker_pub.publish(m_dist); 
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "spline_optimization_example");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    listener = new tf::TransformListener;

    occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 5, true);
    free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 5, true);
    dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5, true);

    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_ ;
    // depth_image_sub_.subscribe(nh, "/camera/depth/image_raw", 5);
    // tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5);

    depth_image_sub_.subscribe(nh, "/depth_topic_2", 5);
    tf::MessageFilter<sensor_msgs::Image> tf_filter_(depth_image_sub_, *listener, "/camera_link", 5);  //camera_depth_optical_frame
    
    tf_filter_.registerCallback(depthImageCallback);

    double resolution;
    pnh.param("resolution", resolution, 0.15);
    edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));

    double distance_threshold_;
    pnh.param("distance_threshold", distance_threshold_, 0.8); //0.5
    
    ROS_INFO("Started spline_optimization_example");

    ros::Publisher global_traj_pub = nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);
    ros::Publisher before_opt_pub = nh.advertise<visualization_msgs::MarkerArray>("before_optimization", 1, true);
    ros::Publisher after_opt_pub = nh.advertise<visualization_msgs::MarkerArray>("after_optimization", 1, true);

    // congtranv
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50, currentPoseCallback);
    ros::Publisher point_pub = nh.advertise<geometry_msgs::Point>("optimization_point", 1);
    ros::Publisher point_target_pub = nh.advertise<std_msgs::Float32MultiArray>("point_target",1);
    ros::Publisher check_last_opt_point_pub = nh.advertise<std_msgs::Bool>("check_last_opt_point",1);

    nh.getParam("/spline_optimization_example/number_of_target", target_num);
    nh.getParam("/spline_optimization_example/target_error", target_error_);
    nh.getParam("/spline_optimization_example/x_pos", x_target);
    nh.getParam("/spline_optimization_example/y_pos", y_target);
    nh.getParam("/spline_optimization_example/z_pos", z_target);
    
    for(int i=0; i<target_num; i++){
        target_array.data.push_back(x_target[i]);
        target_array.data.push_back(y_target[i]);
        target_array.data.push_back(z_target[i]);
    }

    // Set up global trajectory
    // HM: const Eigen::Vector4d limits(0.5, 3, 0.2, 0);
    // DuyNguyen: const Eigen::Vector4d limits(0.7, 4, 0.2, 0);
    const Eigen::Vector4d limits(0.5, 3, 0.2, 0); // ivsr velocity, acceleration, jerk, snap   //A row-vector containing the elements {0.7, 4, 0, 0} 

    ewok::Polynomial3DOptimization<10> po(limits*0.8);//0.8 ??? limits
    //
    typename ewok::Polynomial3DOptimization<10>::Vector3Array vec;   //vec la mang cac vector3

    // congtranv
    for(int i=0; i<target_num; i++)
    {
        vec.push_back(Eigen::Vector3d(x_target[i], y_target[i], z_target[i]));
        std::cout << x_target[i] << ", " << y_target[i] << ", " << z_target[i] << "\n";
    }


    auto traj = po.computeTrajectory(vec);

    visualization_msgs::MarkerArray traj_marker;
    traj->getVisualizationMarkerArray(traj_marker, "trajectory", Eigen::Vector3d(1, 1, 0), 0.5); //100

    global_traj_pub.publish(traj_marker);

    // Set up spline optimization
    // HM: const int num_points = 7;
    const int num_points = 10;
    // DuyNguyen: const double dt = 0.5;
    const double dt = 0.4;

    ewok::UniformBSpline3DOptimization<6> spline_opt(traj, dt);

    for (int i = 0; i < num_points; i++) {
        spline_opt.addControlPoint(vec[0]);
    }

    spline_opt.setNumControlPointsOptimized(num_points);
    spline_opt.setDistanceBuffer(edrb);
    spline_opt.setDistanceThreshold(distance_threshold_);
    spline_opt.setLimits(limits);


    double tc = spline_opt.getClosestTrajectoryTime(Eigen::Vector3d(-3, -5, 1), 2.0);
    ROS_INFO_STREAM("Closest time: " << tc);

    ROS_INFO("Finished setting up data");

    double current_time = 0;

    double total_opt_time = 0;
    int num_iterations = 0;

    ros::Rate r(1.0/dt);

    // congtranv
    ewok::EuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
    while(ros::ok() && !start_reached)
    {
        point_target_pub.publish(target_array);
        //start_reached = checkPosition(1.0, current_pose, targetTransfer(vec[0].x(), vec[0].y(), vec[0].z()));
        start_reached = checkPosition(target_error_, current_pose, targetTransfer(vec[0].x(), vec[0].y(), vec[0].z()));
        // std::cout << "\n" << targetTransfer(vec[0].x(), vec[0].y(), vec[0].z()).pose.position.x << ", " << targetTransfer(vec[0].x(), vec[0].y(), vec[0].z()).pose.position.y << ", " << targetTransfer(vec[0].x(), vec[0].y(), vec[0].z()).pose.position.z << "\n";
        // std::cout << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << "\n";
        ros::spinOnce();
    }
    start_reached = false;
    // while (ros::ok() && current_time < traj->duration()) {
    while (ros::ok() && !start_reached) {
        r.sleep();
        current_time += dt;

        visualization_msgs::MarkerArray before_opt_markers, after_opt_markers;

        spline_opt.getMarkers(before_opt_markers, "before_opt",
                            Eigen::Vector3d(1, 0, 0),
                            Eigen::Vector3d(1, 0, 0));

        auto t1 = std::chrono::high_resolution_clock::now();
        double error = spline_opt.optimize();
        auto t2 = std::chrono::high_resolution_clock::now();

        double miliseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() / 1.0e6;

        total_opt_time += miliseconds;
        num_iterations++;


        ROS_INFO_STREAM("Finished optimization in " << miliseconds << " ms. Error: " << error);

        spline_opt.getMarkers(after_opt_markers, "after_opt",
                            Eigen::Vector3d(0, 1, 0),
                            Eigen::Vector3d(0, 1, 1));

        after_opt_pub.publish(after_opt_markers);

        spline_opt.addLastControlPoint();

        std :: cout << "=============================================" << std::endl;
        std :: cout << "First Control Point: \n" << spline_opt.getFirstOptimizationPoint() << std::endl;
        std :: cout << "=============================================" << std::endl;

        last_ctrl_point.x = spline_opt.getFirstOptimizationPoint().x();
        last_ctrl_point.y = spline_opt.getFirstOptimizationPoint().y();
        last_ctrl_point.z = spline_opt.getFirstOptimizationPoint().z();
        point_pub.publish(last_ctrl_point);
        // DuyNguyen
        start_reached = checkPosition(target_error_, current_pose, targetTransfer(x_target[target_num-1], y_target[target_num-1], z_target[target_num-1]));
        ros::spinOnce();
    }

    while (ros::ok()) {
        r.sleep();
        check_last_opt_point.data = true;
        check_last_opt_point_pub.publish(check_last_opt_point);
        std :: cout << "check_last_opt_point = " << check_last_opt_point.data << std::endl;
        ros::spinOnce();
    }

    f_time.close();
    opt_time.close();
    return 0;
}
