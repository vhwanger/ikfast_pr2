#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ikfast_pr2/arm.h>
#include <boost/lexical_cast.hpp>
using namespace boost;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    //0.248267 -1.074898 0.000000 1.440684 -2.525041 0.438921 2.571189
    //0.248267
    //-1.074898
    //0.000000
    //1.440684
    //-2.525041
    //0.438921
    //2.571189
    //
    //0.715428
    //-0.149809
    //1.000000
    //-1.390838
    //-0.607310
    //0.906194
    //-0.793042
    //
    //-0.329303
    //-0.515434
    //1.000000
    //0.607226
    //2.309950
    //1.433818
    //1.394146
    //
    //-0.462168
    //1.000000
    //0.347828
    //2.248839
    //1.313628
    //2.714702
    //
    //0.125787
    //-0.462168
    //1.000000
    //0.347828
    //2.248839
    //1.313628
    //2.714702
    //
    //-0.029731
    //-0.490309
    //0.684117
    //0.408230
    //2.580378
    //1.447931
    //2.840452
    //
    //0.125733
    //-0.352004
    //0.648999
    //-0.150824
    //-48.016645
    //-0.112334
    //8.997872
    //
    //2.885677
    //-2.344324
    //0.409109
    //-1.409840
    //-2.410434
    //1.792690
    //-2.024487
    //
    //('r_shoulder_pan_joint', 0.2862405231409586),
    //('r_shoulder_lift_joint', 0.33346823872722264),
    //('r_upper_arm_roll_joint', 0.13522256369755992),
    //('r_elbow_flex_joint', -1.2713495942083988),
    //('r_forearm_roll_joint', -0.23342253637528287),
    //('r_wrist_flex_joint', -0.6348326584171256),
    //('r_wrist_roll_joint', 3.0910447360157214),

    goal.trajectory.points[ind].positions[0] =0.2862405231409586;
    goal.trajectory.points[ind].positions[1] =0.33346823872722264;
    goal.trajectory.points[ind].positions[2] =0.13522256369755992;
    goal.trajectory.points[ind].positions[3] =-1.2713495942083988;
    goal.trajectory.points[ind].positions[4] =-0.23342253637528287;
    goal.trajectory.points[ind].positions[5] =-0.6348326584171256;
    goal.trajectory.points[ind].positions[6] =3.0910447360157214;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  Arm arm("right");
  std::vector<double> angles;
  
  //
  //0.449850
  //0.120406
  //0.680000
  //-0.766239
  //-0.773328
  //-1.105439
  //0.059841
  //
  angles.push_back(0.449850);
  angles.push_back(0.120406);
  angles.push_back(0.680000);
  angles.push_back(-0.766239);
  angles.push_back(-0.773328);
  angles.push_back(-1.105439);
  angles.push_back(0.059841);

  arm.sendArmToConfiguration(&angles[0], 3);
  // Start the trajectory
  //arm.startTrajectory(arm.armExtensionTrajectory());
  //// Wait for trajectory completion
  //while(!arm.getState().isDone() && ros::ok())
  //{
  //  usleep(50000);
  //}
}
