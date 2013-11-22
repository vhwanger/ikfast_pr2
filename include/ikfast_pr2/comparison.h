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

class Tester {
    public:
        Tester();
        void run_ik(const sensor_msgs::JointState& msg);
    private:
        IKFastPR2 ik_solver;
        Arm arm;
        int counter;
        int kdl_c;
        int ikfast_c;
        tf::TransformListener listener;
        unsigned int ikfast_time;
        unsigned int kdl_time;

        unsigned int kdl_fk;
        unsigned int ikfast_fk;
};
