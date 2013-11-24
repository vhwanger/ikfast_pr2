#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ikfast_pr2/ik_interface.h>
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <pviz/pviz.h>
#include <tf_conversions/tf_kdl.h>
#include <ikfast_pr2/arm.h>
#include <angles/angles.h>
#include <pr2_collision_checker/sbpl_arm_model.h>

class Tester {
    public:
        Tester();
        void run_ik(const sensor_msgs::JointState& msg);
    private:
        bool run_ikfast_test(const KDL::Frame& wrist_frame, double free_angle);
        bool run_kdl_test(const KDL::Frame& wrist_frame, 
                          const vector<double>& seed_angle);


        IKFastPR2 ik_solver;
        int counter;
        int kdl_c;
        int ikfast_c;
        boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> m_arm_model;
        tf::TransformListener listener;
        unsigned long int ikfast_time;
        unsigned long int kdl_time;
        unsigned long int kdl_fk;
        unsigned long int ikfast_fk;
};
