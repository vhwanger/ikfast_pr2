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
    ROS_INFO("computed obj pose");
    ROS_INFO("%f %f %f %f %f %f",
            obj.x,
            obj.y,
            obj.z,
            obj.roll,
            obj.pitch,
            obj.yaw);

    tf::StampedTransform fk_transform;
    KDL::Frame wrist_frame;
    tf::TransformListener listener;
    listener.waitForTransform("/torso_lift_link", "/r_wrist_roll_link", ros::Time(0), ros::Duration(10));
    listener.lookupTransform("/torso_lift_link", "/r_wrist_roll_link", ros::Time(0), fk_transform);
    tf::TransformTFToKDL(fk_transform, wrist_frame);


    double roll, pitch, yaw;
    wrist_frame.M.GetRPY(roll, pitch, yaw);
    assert(wrist_frame.p.x() == obj.x);
    assert(wrist_frame.p.y() == obj.y);
    assert(wrist_frame.p.z() == obj.z);
    assert(roll == obj.roll);
    assert(pitch == obj.pitch);
    assert(yaw == obj.yaw);

    std::vector<double> ik_angles;
    ik_solver.ik(wrist_frame, msg.position[17], &ik_angles);
    assert(ik_angles[0] == msg.position[18]);
    assert(ik_angles[1] == msg.position[19]);
    assert(ik_angles[2] == msg.position[17]);
    assert(ik_angles[3] == msg.position[21]);
    assert(ik_angles[4] == msg.position[20]);
    assert(ik_angles[5] == msg.position[22]);
    assert(ik_angles[6] == msg.position[23]);
}
int main(int argc, char** argv){
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, run_ik);
    ros::spin();
    return 0;
}
