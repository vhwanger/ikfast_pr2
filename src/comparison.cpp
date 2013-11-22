#include <ikfast_pr2/comparison.h>

using namespace std;


Tester::Tester():arm("right"){ arm.setReferenceFrame("/torso_lift_link");}

void Tester::run_ik(const sensor_msgs::JointState& msg){
    std::vector<double> angles;
    angles.push_back(msg.position[18]);
    angles.push_back(msg.position[19]);
    angles.push_back(msg.position[17]);
    angles.push_back(msg.position[21]);
    angles.push_back(msg.position[20]);
    angles.push_back(msg.position[22]);
    angles.push_back(msg.position[23]);

    tf::StampedTransform fk_transform;
    KDL::Frame wrist_frame;
    listener.waitForTransform("/torso_lift_link", "/r_wrist_roll_link", ros::Time(0), ros::Duration(10));
    listener.lookupTransform("/torso_lift_link", "/r_wrist_roll_link", ros::Time(0), fk_transform);
    tf::TransformTFToKDL(fk_transform, wrist_frame);

    geometry_msgs::Pose pose;
    pose.position.x = wrist_frame.p.x();
    pose.position.y = wrist_frame.p.y();
    pose.position.z = wrist_frame.p.z();
    wrist_frame.M.GetQuaternion(pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w);
    double wroll, wpitch, wyaw;
    wrist_frame.M.GetRPY(wroll, wpitch, wyaw);

    vector<double> ik_angles;
    vector<double> kdl_angles(7,0);
    bool fastik_success = ik_solver.ik(wrist_frame, msg.position[17], &ik_angles);

    if (fastik_success){
        KDL::Frame obj_frame = ik_solver.getKDLObjectState(ik_angles);
        double roll, pitch, yaw;
        obj_frame.M.GetRPY(roll, pitch, yaw);
        assert(fabs(wrist_frame.p.x()-obj_frame.p.x()) < .001);
        assert(fabs(wrist_frame.p.y()-obj_frame.p.y()) < .001);
        assert(fabs(wrist_frame.p.z()-obj_frame.p.z()) < .001);
        assert(fabs(wroll-roll) < .0001);
        assert(fabs(wpitch-pitch) < .0001);
        assert(fabs(wyaw-yaw) < .0001);
    } else {
        ROS_ERROR("fast ik failed");
    }


    bool kdl_success = false;
    kdl_success = arm.computeIK(pose, angles, kdl_angles);
    sleep(.1);

    ROS_INFO("kdl %d      fast_ik %d", kdl_success, fastik_success);
}
int main(int argc, char** argv){
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    Tester tester;
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, &Tester::run_ik, &tester);
    sleep(1);
    ros::spin();
    return 0;
}
