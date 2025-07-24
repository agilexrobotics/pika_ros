#include <ros/ros.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

class MarkerPublish{
public:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Subscriber subscriber;

    MarkerPublish() {
        publisher = nh.advertise<visualization_msgs::Marker>("pika_marker", 2000);
        subscriber = nh.subscribe<geometry_msgs::PoseStamped>("/vive_pose", 1, &MarkerPublish::handler, this, ros::TransportHints().tcpNoDelay());
    }

    Eigen::Affine3f pose2affine(const geometry_msgs::Pose &pose)
    {
        double x, y, z, roll, pitch, yaw;
        x = pose.position.x;
        y = pose.position.y;
        z = pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void handler(const geometry_msgs::PoseStamped::ConstPtr& msg){
        Eigen::Affine3f transFront = pcl::getTransformation(-0.3, -0.108, -0.125, M_PI / 2.0, 0, M_PI / 2.0);
        Eigen::Affine3f transback = pose2affine(msg->pose);
        Eigen::Affine3f transFinal = transback * transFront;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles (transFinal, x, y, z, roll, pitch, yaw);
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time().now();
        marker.header.frame_id = "map";
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        marker.mesh_resource = "file:///home/agilex/pika_ros/src/sensor_tools/meshes/pika.STL";
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.type = 10;
        marker.action = 3;
        publisher.publish(marker);

        marker.header.stamp = ros::Time().now();
        marker.header.frame_id = "map";
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        marker.mesh_resource = "file:///home/agilex/pika_ros/src/sensor_tools/meshes/pika.STL";
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.type = 10;
        marker.action = 0;
        publisher.publish(marker);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_publish");
    ros::NodeHandle nh;
    MarkerPublish markerPublish;
    ros::spin();
    return 0;
}
