///  Copyright 2015 Matthew Swartwout

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_traj_streamer/trajAction.h>
#include <my_interesting_moves/interesting_moves.h>
#include <move_doer/move_doer.h>

MoveDoer::MoveDoer() : action_client("trajActionServer", true)
{
    g_count = 0;
}

///  This function will be called once when the goal completes
///  this is optional, but it is a convenient way to get access to the "result" message sent by the server
void MoveDoer::doneCb(const actionlib::SimpleClientGoalState& state,
                      const baxter_traj_streamer::trajResultConstPtr& result)
{
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}

void MoveDoer::sendMoveToServer(baxter_traj_streamer::trajGoal &goal)
{
    g_count++;
    goal.traj_id = g_count;  /// this merely sequentially numbers the goals sent
    ROS_INFO("sending traj_id %d", g_count);

    action_client.sendGoal(goal, &doneCb);

    bool finished_before_timeout = action_client.waitForResult(ros::Duration(40.0));
    if (!finished_before_timeout)
    {
        ROS_WARN("giving up waiting on result for goal number %d", g_count);
    }
    else
    {
        ROS_INFO("finished before timeout");
    }
}

bool MoveDoer::startServer()
{
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0));  /// wait for up to 5 seconds
    int fail_count = 0;

    if (!server_exists)
    {
        ROS_WARN("could not connect to server; will try again");
        while (fail_count < 10)
        {
            server_exists = action_client.waitForServer(ros::Duration(5.0));
            if (server_exists)
            {
                ROS_INFO("connected to action server");  /// if here, then we connected to the server;
                return true;
            }

            fail_count++;
        }
        return false;
    }

    ROS_INFO("connected to action server");  /// if here, then we connected to the server;
    return true;
}

void MoveDoer::doMoves()
{
    trajectory_msgs::JointTrajectory des_trajectory;  /// an empty trajectory

    /// Baxter assumes the position
    move_maker.set_move_ready(des_trajectory);
    goal.trajectory = des_trajectory;
    sendMoveToServer(goal);

    /// Baxter gives you a friendly wave
    move_maker.set_move_wave(des_trajectory);
    goal.trajectory = des_trajectory;
    sendMoveToServer(goal);

    /// Baxter is displeased with what you've done
    move_maker.set_move_tsk_tsk(des_trajectory);
    goal.trajectory = des_trajectory;
    sendMoveToServer(goal);

    /// Baxter karate chops you in the face
    move_maker.set_move_chop(des_trajectory);
    goal.trajectory = des_trajectory;
    sendMoveToServer(goal);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_doer_node");
    MoveDoer move_doer;

    /// If we can start the server, do the moves
    if (move_doer.startServer())
    {
        move_doer.doMoves();
    }

    return 0;
}
