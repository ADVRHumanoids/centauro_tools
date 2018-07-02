#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "centauro_polygon_publisher");
    ros::NodeHandle nh, nh_private("~");
    
    std::string tf_prefix = nh_private.param("tf_prefix", std::string());
    int swing_foot = nh_private.param("swing_foot", 1);
    auto marker_pub = nh.advertise<visualization_msgs::Marker>("com_projection", 1);
    auto polypub = nh.advertise<geometry_msgs::PolygonStamped>("support_polygon", 1);
    
    std::vector<std::string> wheels = {"wheel_1", "wheel_2", "wheel_4", "wheel_3"};
    
    tf::TransformListener listener;
    ros::Rate rate(50);
    
    while(ros::ok())
    {
        /* Compute polygon */
        geometry_msgs::PolygonStamped polymsg;
        double ground_plane_height = 0;
        int num_stance = 0;
        for(int i = 0; i < wheels.size(); i++)
        {
            if(i == swing_foot) continue;
            
            num_stance++;
            
            tf::StampedTransform transform;
            try{
                listener.lookupTransform(tf_prefix + "/world_odom",
                                         tf_prefix + "/" + wheels[i],
                                         ros::Time(0),
                                         transform);
                
                geometry_msgs::Point32 wheel_pos;
                wheel_pos.x = transform.getOrigin().getX();
                wheel_pos.y = transform.getOrigin().getY();
                wheel_pos.z = transform.getOrigin().getZ();
                ground_plane_height += wheel_pos.z;
                polymsg.polygon.points.push_back(wheel_pos);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(.1).sleep();
            }
        }
        
        ground_plane_height /= num_stance;
        polymsg.header.frame_id = tf_prefix + "/world_odom";
        polymsg.header.stamp = ros::Time::now();
        polypub.publish(polymsg);
        
        /* Publish COM projection */
        tf::StampedTransform transform;
        try{
            listener.lookupTransform(tf_prefix + "/world_odom",
                                        tf_prefix + "/com",
                                        ros::Time(0),
                                        transform);
            
            visualization_msgs::Marker com_marker;
            com_marker.header.frame_id = tf_prefix + "/world_odom";
            com_marker.header.stamp = polymsg.header.stamp;
            com_marker.ns = "centauro_polygon";
            com_marker.id = 0;
            com_marker.type = visualization_msgs::Marker::CYLINDER;
            com_marker.action = visualization_msgs::Marker::ADD;
            com_marker.pose.position.x = transform.getOrigin().getX();
            com_marker.pose.position.y = transform.getOrigin().getY();
            com_marker.pose.position.z = ground_plane_height;
            com_marker.pose.orientation.x = 0.;
            com_marker.pose.orientation.y = 0.;
            com_marker.pose.orientation.z = 0.;
            com_marker.pose.orientation.w = 1.;
            com_marker.scale.x = 0.03;
            com_marker.scale.y = 0.03;
            com_marker.scale.z = 0.003;
            com_marker.color.a = 1.0; // Don't forget to set the alpha!
            com_marker.color.r = 1.0;
            com_marker.color.g = 0.0;
            com_marker.color.b = 0.0;
            
            marker_pub.publish(com_marker);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(.1).sleep();
        }

        rate.sleep();
  }
    
    return 0;
    
}