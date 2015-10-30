#include <ros/ros.h>
#include <baxter_traj_streamer/trajAction.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <actionlib/client/simple_action_client.h>

class MoveDoer {
public:
    MoveDoer();
    static void doneCb(const actionlib::SimpleClientGoalState& state,
            const baxter_traj_streamer::trajResultConstPtr& result);
    void sendMoveToServer(baxter_traj_streamer::trajGoal &goal);
    bool startServer();
    void doMoves();

private:
    int g_count;
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client;//("trajActionServer", true);
    baxter_traj_streamer::trajGoal goal; 
    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    InterestingMoves move_maker;
};
