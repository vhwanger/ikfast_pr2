#include <ikfast_pr2/ik_interface.h>
#include <ros/ros.h>
#include <pviz/pviz.h>
#include <vector>
#include <stdio.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test");

    IKFastPR2 ik_solver;

       // ('r_upper_arm_roll_joint', 0.7782454174408036),
       // ('r_shoulder_pan_joint', 0.5534485508825238),
       // ('r_shoulder_lift_joint', -0.13552601496387404),
       // ('r_forearm_roll_joint', -44.58961020120552),
       // ('r_elbow_flex_joint', -0.5867289000756961),
       // ('r_wrist_flex_joint', -0.00029856768463931704),
       // ('r_wrist_roll_joint', -0.7870669170304522),
       //     ('r_upper_arm_roll_joint', -0.013426360347324584),
       //         ('r_shoulder_pan_joint', 0.055843033505895145),
       //             ('r_shoulder_lift_joint', 0.39082359659204285),
       //                 ('r_forearm_roll_joint', -43.90013077184178),
       //                     ('r_elbow_flex_joint', -1.6193783843743996),
       //                         ('r_wrist_flex_joint', -0.42816528794405073),
       //                             ('r_wrist_roll_joint',
       //                             -0.27566307017981995),
    std::vector<double> angles;
    angles.push_back(.055843);
    angles.push_back(.390823);
    angles.push_back(-.013426);
    angles.push_back(-1.61937);
    angles.push_back(-43.9);
    angles.push_back(-.428165);
    angles.push_back(-.275663);
    ObjectPose obj = ik_solver.getRightArmObjectPose(angles);
    printf("obj %f %f %f %f %f %f\n", 
           obj.x,
           obj.y,
           obj.z,
           obj.roll,
           obj.pitch,
           obj.yaw);
    std::vector<double> pose;
    pose.push_back(obj.x);
    pose.push_back(obj.y);
    pose.push_back(obj.z);
    pose.push_back(obj.roll);
    pose.push_back(obj.pitch);
    pose.push_back(obj.yaw);

    std::vector<double> r_arm;
    r_arm = ik_solver.getRightArmIKFirstSoln(obj, angles[2]);
    if (r_arm.size()){
    printf("ik angles %f %f %f %f %f %f %f\n", 
           r_arm[0],
           r_arm[1],
           r_arm[2],
           r_arm[3],
           r_arm[4],
           r_arm[5],
           r_arm[6]);
    }

    printf("back through fk");
    ObjectPose second_obj = ik_solver.getRightArmObjectPose(r_arm);
    printf("%f %f %f %f %f %f\n", 
            second_obj.x,
            second_obj.y,
            second_obj.z,
            second_obj.roll,
            second_obj.pitch,
            second_obj.yaw);
    


    PViz pviz;
    BodyPose bp;
    bp.x = 0;
    bp.y = 0;
    bp.z = 0;
    bp.theta = 0;
    pviz.setReferenceFrame("torso_lift_link");
    pviz.visualizePose(pose, "endeffector");
    pviz.visualizeRobot(angles, angles, bp, 150, "robot", 0);
    return 0;
}
