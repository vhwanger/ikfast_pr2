#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ikfast_pr2/ik_interface.h>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <pviz/pviz.h>
#include <tf_conversions/tf_kdl.h>


void run_ik(const sensor_msgs::JointState& msg){
    std::vector<double> angles;
    PViz pviz;
    pviz.setReferenceFrame("/torso_lift_link");
    sleep(2);
    angles.push_back(msg.position[18]);
    angles.push_back(msg.position[19]);
    angles.push_back(msg.position[17]);
    angles.push_back(msg.position[21]);
    angles.push_back(msg.position[20]);
    angles.push_back(msg.position[22]);
    angles.push_back(msg.position[23]);
    ROS_INFO("angles %f\n%f\n%f\n%f\n%f\n%f\n%f",
              msg.position[18],
              msg.position[19],
              msg.position[17],
              msg.position[21],
              msg.position[20],
              msg.position[22],
              msg.position[23]);
    IKFastPR2 ik_solver;
    ObjectState obj;
    obj = ik_solver.getRightArmObjectState(angles);
    ROS_INFO("obj pose");
    ROS_INFO("%f %f %f %f %f %f",
            obj.x,
            obj.y,
            obj.z,
            obj.roll,
            obj.pitch,
            obj.yaw);



    pviz.visualizeSphere(obj.x, obj.y, obj.z, .02, 1, "sphere",0);

    ik_solver.ik(obj, msg.position[17], &angles);
    BodyPose bp;
    bp.x = 0;
    bp.y = 0;
    bp.z = 0;
    bp.theta = 0;
    pviz.setReferenceFrame("/base_link");
    std::vector<double> left(7,0);
    pviz.visualizeRobot(angles, left,bp, 150, "robot", 1);
    ROS_INFO("angles %f\n%f\n%f\n%f\n%f\n%f\n%f",
              angles[0],
              angles[1],
              angles[2],
              angles[3],
              angles[4],
              angles[5],
              angles[6]);
    pviz.setReferenceFrame("/torso_lift_link");

    obj = ik_solver.getRightArmObjectState(angles);
    ROS_INFO("obj pose");
    ROS_INFO("%f %f %f %f %f %f",
            obj.x,
            obj.y,
            obj.z,
            obj.roll,
            obj.pitch,
            obj.yaw);

    KDL::Frame kdl_frame = ik_solver.getKDLObjectState(angles);
    tf::Transform my_tf;
    static tf::TransformBroadcaster tf_b;
    tf::TransformKDLToTF(kdl_frame, my_tf);
    tf_b.sendTransform(tf::StampedTransform(my_tf, ros::Time::now(), "/torso_lift_link", "fastik_end_frame"));



    pviz.visualizeSphere(obj.x, obj.y, obj.z, .02, 1, "sphere",3);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("/torso_lift_link", "/r_gripper_tool_frame", ros::Time(0), ros::Duration(10.0) );
    listener.lookupTransform("/torso_lift_link", "/r_gripper_tool_frame", ros::Time(0), transform);
    ROS_INFO("tf got %f %f %f", 
            transform.getOrigin().x(),
            transform.getOrigin().y(),
            transform.getOrigin().z());
    //pviz.visualizeSphere(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), .02, 150, "sphere",1);
    sleep(1);
}
int main(int argc, char** argv){
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, run_ik);
    ros::spin();
    return 0;
}
