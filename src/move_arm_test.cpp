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
    ROS_INFO("wrist frame %f %f %f", wrist_frame.p.x(), wrist_frame.p.y(), wrist_frame.p.z());
    wrist_frame.M.GetRPY(roll, pitch, yaw);
    assert(fabs(wrist_frame.p.x()-obj.x) < .0001);
    assert(fabs(wrist_frame.p.y()-obj.y) < .0001);
    assert(fabs(wrist_frame.p.z()-obj.z) < .0001);
    assert(fabs(roll-obj.roll) < .0001);
    assert(fabs(pitch-obj.pitch) < .0001);
    assert(fabs(yaw-obj.yaw) < .0001);

    std::vector<double> ik_angles;
    ik_solver.ik(wrist_frame, msg.position[17], &ik_angles);
    assert(fabs(ik_angles[0]-msg.position[18]) < .0001);
    assert(fabs(ik_angles[1]-msg.position[19]) < .0001);
    assert(fabs(ik_angles[2]-msg.position[17]) < .0001);
    assert(fabs(ik_angles[3]-msg.position[21]) < .0001);
    assert(fabs(ik_angles[4]-msg.position[20]) < .0001);
    assert(fabs(ik_angles[5]-msg.position[22]) < .0001);
    assert(fabs(ik_angles[6]-msg.position[23]) < .0001);
}
int main(int argc, char** argv){
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, run_ik);
    ros::spin();
    return 0;
}
