#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cartesian_interface/ResetWorld.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "centauro_reset_world");
    ros::NodeHandle nh("cartesian"), nh_private("~");
    
    std::string tf_prefix = nh_private.param("tf_prefix", std::string("ci"));
    std::string tf_prefix_slash = tf_prefix == "" ? tf_prefix : tf_prefix + "/";
    
    
    auto reset_srv = nh.serviceClient<cartesian_interface::ResetWorld>("reset_world");
    reset_srv.waitForExistence();
    
    
    
    std::vector<std::string> wheels = {"wheel_1", "wheel_2", "wheel_4", "wheel_3"};
    
    tf::TransformListener listener;
    
    if(!listener.waitForTransform(tf_prefix_slash + wheels[0], 
                              tf_prefix_slash + "world_odom", 
                              ros::Time(0), 
                              ros::Duration(1.0)))
    {
        exit(EXIT_FAILURE);
    }
    
    if(!listener.waitForTransform(tf_prefix_slash + wheels[1], 
                              tf_prefix_slash + "world_odom", 
                              ros::Time(0), 
                              ros::Duration(1.0)))
    {
        exit(EXIT_FAILURE);
    }
    
    tf::StampedTransform tf_wheel1, tf_wheel2;
    listener.lookupTransform(tf_prefix_slash + "world_odom",
                             tf_prefix_slash + wheels[0],
                            ros::Time(0),
                            tf_wheel1);
    
    listener.lookupTransform(tf_prefix + "/world_odom",
                            tf_prefix + "/" + wheels[1],
                            ros::Time(0),
                            tf_wheel2);
    
    Eigen::Affine3d T_1, T_2;
    tf::transformTFToEigen(tf_wheel1, T_1);
    tf::transformTFToEigen(tf_wheel2, T_2);
    
    Eigen::Affine3d w_T_nw;
    w_T_nw.setIdentity();
    w_T_nw.translation() = 0.5*(T_1.translation() + T_2.translation());
    
    Eigen::Vector3d ux, uy, uz;
    uz << 0, 0, 1;
    uy = T_1.translation() - T_2.translation();
    uy /= uy.norm();
    ux = uy.cross(uz);
    
    w_T_nw.linear() << ux, uy, uz;
    
    geometry_msgs::Pose pose_nw;
    tf::poseEigenToMsg(w_T_nw, pose_nw);
    
    cartesian_interface::ResetWorldRequest req;
    cartesian_interface::ResetWorldResponse res;
    req.new_world = pose_nw;
    
    if(!reset_srv.call(req, res) || !res.success)
    {
        std::cout << res.message << std::endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        exit(EXIT_SUCCESS);
    }
    
}