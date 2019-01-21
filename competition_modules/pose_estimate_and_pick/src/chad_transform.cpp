#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <cmath>

ros::Publisher pub_pose;

geometry_msgs::PoseStamped pose_cam;
geometry_msgs::Point pose_base;

bool lock = true;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr msg){
  if (lock){
  lock = false;
  pose_cam = *msg;
  tf::Transform cam2target;
  tf::Matrix3x3 R_target(1, 0, 0, 0, 1, 0, 0, 0, 1);
  ROS_INFO("target : %f \t %f \t %f \n",pose_cam.pose.position.x, pose_cam.pose.position.y,pose_cam.pose.position.z);
  tf::Vector3 V_target(pose_cam.pose.position.x, pose_cam.pose.position.y, pose_cam.pose.position.z);
  tf::Quaternion q;
  R_target.getRotation(q);
  cam2target.setOrigin(V_target);
  cam2target.setRotation(q);

  tf::TransformListener listener1;
  tf::TransformListener listener2;
  tf::StampedTransform Transform1;
  tf::StampedTransform Transform2;

  try{
    //ros::Time now = ros::Time::now();
    listener1.waitForTransform("car_base", "camera_link", ros::Time(0), ros::Duration(3.0));
    listener1.lookupTransform("car_base", "camera_link", ros::Time(0), Transform1);
    listener2.waitForTransform("car_base", "base_link", ros::Time(0), ros::Duration(3.0));
    listener2.lookupTransform("car_base", "base_link", ros::Time(0), Transform2);
    }
  catch (tf::TransformException ex){
    //ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  tf::Transform car2cam, car2arm, arm2cam, arm2goal;

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
  tf::Matrix3x3 mat(q_car2cam);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  //printf("euler : %f \t %f \t %f \n",roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
  //printf("car2cam : %f, %f, %f\n",Transform1.getOrigin().x(), Transform1.getOrigin().y(), Transform1.getOrigin().z());
  //printf("car2arm : %f, %f, %f\n",Transform2.getOrigin().x(), Transform2.getOrigin().y(), Transform2.getOrigin().z());
  car2cam.setOrigin(v_car2cam); car2cam.setRotation(q_car2cam);
  car2arm.setOrigin(v_car2arm); car2arm.setRotation(q_car2arm);
  arm2cam =car2arm.inverse() * car2cam;

  ROS_INFO("arm2cam : %f \t %f \t %f \n",arm2cam.getOrigin().x(), arm2cam.getOrigin().y(), arm2cam.getOrigin().z());

  arm2goal = car2arm.inverse() * car2cam * cam2target;
  pose_base.x = arm2goal.getOrigin().x();
  pose_base.y = arm2goal.getOrigin().y();
  pose_base.z = arm2goal.getOrigin().z();

  ROS_INFO("pose : %f \t %f \t %f \n",pose_base.x, pose_base.y, pose_base.z);

  pub_pose.publish(pose_base);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "chad_transform");
  ros::NodeHandle nh;

  // declare subscriber and publisher
  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped> ("/camera/product_pose", 5, &pose_cb);
  pub_pose = nh.advertise<geometry_msgs::Point> ("sis_competition/chadlin/object", 2);

  ros::spin();
}
