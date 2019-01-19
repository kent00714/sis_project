#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

ros::Publisher pub_pose;

geometry_msgs::Point pose_cam;
geometry_msgs::Point pose_base;
tf::TransformListener listener;
tf::StampedTransform Transform1;
tf::StampedTransform Transform2;

void pose_cb(const geometry_msgs::Point::ConstPtr msg){
  pose_cam = *msg;
  tf::Transform target;
  tf::Matrix3x3 R_target(1, 0, 0, 0, 1, 0, 0, 0, 1);
  tf::Vector3 V_target(pose_cam.x, pose_cam.y, pose_cam.z);
  target.setOrigin(V_target);
  target.setRotation(R_target);

  try{
    ros::Time now = ros::Time::now();
    listener.waitForTransform("car_base", "camera_link", now, ros::Duration(3.0));
    listener.lookupTransform("car_base", "camera_link", now, Transform1);
    listener.waitForTransform("car_base", "base_link", now, ros::Duration(3.0));
    listener.lookupTransform("car_base", "base_link", now, Transform2);
    }
  catch (tf::TransformException ex){
    //ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Transform car2cam, car2arm, cam2arm, arm2goal;

  // start to combine 2 transform matrix
  tf::Quaternion q_car2cam(Transform1.getRotation().x(),
                           Transform1.getRotation().y(),
                           Transform1.getRotation().z(),
                           Transform1.getRotation().w());
  tf::Quaternion q_car2arm(Transform2.getRotation().x(),
                           Transform2.getRotation().y(),
                           Transform2.getRotation().z(),
                           Transform2.getRotation().w());
  tf::Vector3 v_car2cam(Transform1.getOrigin().x(),
                        Transform1.getOrigin().y(),
                        Transform1.getOrigin().z());
  tf::Vector3 v_car2arm(Transform2.getOrigin().x(),
                        Transform2.getOrigin().y(),
                        Transform2.getOrigin().z());

  car2cam.setOrigin(v_car2cam); car2cam.setRotation(q_car2cam);
  car2arm.setOrigin(v_car2arm); car2arm.setRotation(q_car2arm);
  cam2arm = car2cam.inverse() * car2arm;
  arm2goal = car2arm.inverse() * car2cam * target;

  pose_base.x = arm2goal.getOrigin().x();
  pose_base.x = arm2goal.getOrigin().y();
  pose_base.x = arm2goal.getOrigin().z();

  pub_pose.publish(pose_base);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chad_transform");
  ros::NodeHandle nh;

  // declare subscriber and publisher
  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::Point> ("sis_competition/chadlin/pose", 1000, pose_cb);
  pub_pose = nh.advertise<geometry_msgs::Point> ("sis_competition/chadlin/object", 2);

  ros::spin();
}
