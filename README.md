ikfast_pr2
==========

An inverse kinematics solver for the PR2 using OpenRave's IKFast package. This IK solver is particularly useful due its speed (on the order of 100 microseconds) and analytic solutions (with 97% success rate, according to the OpenRave website). To use this class, add the following to the appropriate file and link the library to your binary: 

        #include <ikfast_pr2/ik_interface.h>

To use the solver, instantiate an object

       IKFastPR2 solver;

These are the functions you'll probably end up using:

        bool ikRightArm(const KDL::Frame& wrist_frame,
                        double free_angle,
                        std::vector<double>* angles,
                        bool search_free_angle=false);
        bool ikLeftArm(const KDL::Frame& wrist_frame,
                       double free_angle,
                       std::vector<double>* angles,
                       bool search_free_angle=false);

        KDL::Frame fkRightArm(const std::vector<double> arm_angles);
        KDL::Frame fkLeftArm(const std::vector<double> arm_angles);

The inverse kinematic functions require you to specify a free_angle corresponding to the upper arm roll angle. The search_free_angle parameter specifies whether to try and find any free angle that returns a solution. It searches using the specified free angle, then tries +/- .01 radians, and continues until it tries all angles. It then returns a solution to the angles vector.

The ik function takes in a pose in the torso_lift_link frame. The forward kinematic function returns a solution in the wrist_roll_link frame. If you find that ik fails more frequently then it should, make sure the pose is in the correct frame.
