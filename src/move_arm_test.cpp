#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ikfast_pr2/ik_interface.h>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <pviz/pviz.h>
#include <tf_conversions/tf_kdl.h>
#include <angles/angles.h>

using namespace std;

void run_ik(const sensor_msgs::JointState& msg){
    std::vector<double> angles;
    PViz pviz;
    pviz.setReferenceFrame("/base_link");
    angles.push_back(msg.position[18]);
    angles.push_back(msg.position[19]);
    angles.push_back(msg.position[17]);
    angles.push_back(msg.position[21]);
    angles.push_back(msg.position[20]);
    angles.push_back(msg.position[22]);
    angles.push_back(msg.position[23]);
    ROS_INFO("angles %f\n%f\n%f\n%f\n%f\n%f\n%f",
              angles::normalize_angle(msg.position[18]),
              angles::normalize_angle(msg.position[19]),
              angles::normalize_angle(msg.position[17]),
              angles::normalize_angle(msg.position[21]),
              angles::normalize_angle(msg.position[20]),
              angles::normalize_angle(msg.position[22]),
              angles::normalize_angle(msg.position[23]));
    IKFastPR2 ik_solver;
    KDL::Frame obj_frame;
    obj_frame = ik_solver.fkRightArm(angles);
    double roll, pitch, yaw;
    obj_frame.M.GetRPY(roll, pitch, yaw);

    ROS_INFO("computed obj pose");
    ROS_INFO("%f %f %f %f %f %f",
            obj_frame.p.x(),
            obj_frame.p.y(),
            obj_frame.p.z(),
            roll,
            pitch,
            yaw);

    tf::StampedTransform fk_transform;
    KDL::Frame wrist_frame;
    tf::TransformListener listener;
    listener.waitForTransform("/torso_lift_link", "/r_wrist_roll_link", ros::Time(0), ros::Duration(10));
    listener.lookupTransform("/torso_lift_link", "/r_wrist_roll_link", ros::Time(0), fk_transform);
    tf::transformTFToKDL(fk_transform, wrist_frame);


    double roll2, pitch2, yaw2;
    wrist_frame.M.GetRPY(roll2, pitch2, yaw2);
    ROS_INFO("wrist frame %f %f %f (%f %f %f)", 
             wrist_frame.p.x(), wrist_frame.p.y(), wrist_frame.p.z(),
             roll, pitch, yaw);
    assert(fabs(wrist_frame.p.x()-obj_frame.p.x()) < .001);
    assert(fabs(wrist_frame.p.y()-obj_frame.p.y()) < .001);
    assert(fabs(wrist_frame.p.z()-obj_frame.p.z()) < .001);
    assert(fabs(roll2-roll) < .0001);
    assert(fabs(pitch2-pitch) < .0001);
    assert(fabs(yaw2-yaw) < .0001);

    std::vector<double> ik_angles;

    vector<vector<double> > all_soln;
    ik_solver.ikAllSolnRightArm(wrist_frame, msg.position[17], &all_soln);
    BodyPose bp;
    bp.x = 0; bp.y = 0; bp.z = 0; bp.theta = 0;

    //for (int i=0; i < all_soln.size(); i++){
    //    vector<double> test_ang = all_soln[i];
    //    pviz.visualizeRobot(test_ang, test_ang, bp, 150, "blah", 0);
    //    ROS_INFO("currently visualizing %f %f %f %f %f %f %f",
    //            test_ang[0],
    //            test_ang[1],
    //            test_ang[2],
    //            test_ang[3],
    //            test_ang[4],
    //            test_ang[5],
    //            test_ang[6]);
    //    char shit;
    //    std::cin >> shit;
    //}

    if (ik_solver.ikRightArm(wrist_frame, msg.position[17], &ik_angles)){
        obj_frame = ik_solver.fkRightArm(ik_angles);
        obj_frame.M.GetRPY(roll, pitch, yaw);
        ROS_INFO("computed obj_frame %f %f %f %f %f %f", obj_frame.p.x(), obj_frame.p.y(), obj_frame.p.z(), roll, pitch, yaw);
        ROS_INFO("successful angle %f %f %f %f %f %f %f",
                ik_angles[0],
                ik_angles[1],
                ik_angles[2],
                ik_angles[3],
                ik_angles[4],
                ik_angles[5],
                ik_angles[6]);
        assert(fabs(wrist_frame.p.x()-obj_frame.p.x()) < .001);
        assert(fabs(wrist_frame.p.y()-obj_frame.p.y()) < .001);
        assert(fabs(wrist_frame.p.z()-obj_frame.p.z()) < .001);
        assert(fabs(roll2-roll) < .0001);
        assert(fabs(pitch2-pitch) < .0001);
        assert(fabs(yaw2-yaw) < .0001);
        ROS_INFO("test succeeded!");
        pviz.visualizeRobot(ik_angles, ik_angles, bp, 150, "blah", 0);
    } else {
        ROS_INFO("ik failed");
    }
}
int main(int argc, char** argv){
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, run_ik);
    ros::spin();
    return 0;
}
