#include <ikfast_pr2/comparison_left.h>

using namespace std;


Tester::Tester():arm("left"),counter(0),kdl_c(0),ikfast_c(0),ikfast_time(0){ arm.setReferenceFrame("/torso_lift_link");}

void Tester::run_ik(const sensor_msgs::JointState& msg){
    std::vector<double> angles;
    angles.push_back(msg.position[32]);
    angles.push_back(msg.position[33]);
    angles.push_back(msg.position[31]);
    angles.push_back(msg.position[35]);
    angles.push_back(msg.position[34]);
    angles.push_back(msg.position[36]);
    angles.push_back(msg.position[37]);

    tf::StampedTransform fk_transform;
    KDL::Frame wrist_frame;
    listener.waitForTransform("/torso_lift_link", "/l_wrist_roll_link", ros::Time(0), ros::Duration(10));
    listener.lookupTransform("/torso_lift_link", "/l_wrist_roll_link", ros::Time(0), fk_transform);
    tf::transformTFToKDL(fk_transform, wrist_frame);

    double wroll, wpitch, wyaw;
    wrist_frame.M.GetRPY(wroll, wpitch, wyaw);

    vector<double> ik_angles;
    vector<double> kdl_angles(7,0);
    struct timeval tv_b;
    struct timeval tv_a;
    gettimeofday(&tv_b, NULL);
    double before = tv_b.tv_usec + (tv_b.tv_sec * 1000);
    bool fastik_success = ik_solver.ikLeftArm(wrist_frame, msg.position[17], &ik_angles);
    gettimeofday(&tv_a, NULL);
    double after = tv_a.tv_usec + (tv_a.tv_sec * 1000);
    ikfast_time += after - before;

    if (fastik_success){
        KDL::Frame obj_frame = ik_solver.fkLeftArm(ik_angles);
        double roll, pitch, yaw;
        obj_frame.M.GetRPY(roll, pitch, yaw);
        assert(fabs(wrist_frame.p.x()-obj_frame.p.x()) < .001);
        assert(fabs(wrist_frame.p.y()-obj_frame.p.y()) < .001);
        assert(fabs(wrist_frame.p.z()-obj_frame.p.z()) < .001);
        assert(fabs(wroll-roll) < .0001);
        assert(fabs(wpitch-pitch) < .0001);
        assert(fabs(wyaw-yaw) < .0001);
        ikfast_c++;
    } else {
        ROS_ERROR("fast ik failed");
    }


    gettimeofday(&tv_b, NULL);
    before = tv_b.tv_usec + (tv_b.tv_sec * 1000);
    geometry_msgs::Pose pose;
    pose.position.x = wrist_frame.p.x();
    pose.position.y = wrist_frame.p.y();
    pose.position.z = wrist_frame.p.z();
    wrist_frame.M.GetQuaternion(pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w);
    bool kdl_success = arm.computeIK(pose, angles, kdl_angles);
    gettimeofday(&tv_a, NULL);
    after = tv_a.tv_usec + (tv_a.tv_sec * 1000);
    kdl_time += after - before;

    if (kdl_success){
        kdl_c++;
    }
    sleep(.1);

    ROS_INFO("%d: kdl %d      fast_ik %d", counter, kdl_success, fastik_success);

    counter++;
    if (counter == 1000){
        ROS_INFO("kdl: %d, fastik %d", kdl_c, ikfast_c);
        ROS_INFO("kdl: %f, fastik %f", kdl_time, ikfast_time);
        sleep(100);
    }
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
