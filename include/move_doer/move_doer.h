/// Copyright 2015 Matthew Swartwout

#ifndef MOVE_DOER_MOVE_DOER_H
#define MOVE_DOER_MOVE_DOER_H

#include <ros/ros.h>
#include <baxter_traj_streamer/trajAction.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <actionlib/client/simple_action_client.h>

class MoveDoer
{
public:
    /**
     * Constructor for MoveDoer class
     */    
    MoveDoer();

    /**
     * Callback function for ActionServer client
     * @param state The goal state
     * @param result The result returned by the server
     */
    static void doneCb(const actionlib::SimpleClientGoalState& state,
            const baxter_traj_streamer::trajResultConstPtr& result);

    /**
     * Function to start the ActionServer and connect
     */
    bool startServer();

    /**
     * Function to get and execute move trajectories
     */
    void doMoves();

private:
    int g_count;
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client;
    baxter_traj_streamer::trajGoal goal;
    InterestingMoves move_maker;

    void sendMoveToServer(baxter_traj_streamer::trajGoal &goal);
};

#endif  // MOVE_DOER_MOVE_DOER_H
