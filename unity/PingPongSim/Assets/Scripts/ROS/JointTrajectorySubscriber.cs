/*
Based on work done by Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class JointTrajectorySubscriber : UnitySubscriber<MessageTypes.Trajectory.JointTrajectory>
    {

        public Robot.JointTrajectoryController controller;

        private MessageTypes.Trajectory.JointTrajectory mostRecentMessage;
        private Robot.JointTrajectory trajectory = new Robot.JointTrajectory();

        private bool isMessageReceived;

        protected override void Start()
		{
			base.Start();
		}

        private void Update()
        {
            if (isMessageReceived)
            {
                ProcessMessage();
                isMessageReceived = false;
            }
        }

        protected override void ReceiveMessage(MessageTypes.Trajectory.JointTrajectory message)
        {
            mostRecentMessage = message;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            trajectory.points.Clear();
            trajectory.joint_names.Clear();
            trajectory.start_time = Time.fixedTime;

            foreach (string joint in mostRecentMessage.joint_names)
            {
                trajectory.joint_names.Add(joint);
            }

            foreach (MessageTypes.Trajectory.JointTrajectoryPoint point in mostRecentMessage.points)
            {
                Robot.JointTrajectoryPoint joint_traj_point = new Robot.JointTrajectoryPoint();
                for(int i = 0; i < trajectory.joint_names.Count; i++) {
                    joint_traj_point.positions.Add(System.Convert.ToSingle(-Mathf.Rad2Deg*point.positions[i]));
                    joint_traj_point.velocities.Add(System.Convert.ToSingle(-Mathf.Rad2Deg*point.velocities[i]));
                }
                joint_traj_point.time_from_start = point.time_from_start.nsecs/1000;
                trajectory.points.Add(joint_traj_point);
            }
            controller.followTrajectory(trajectory);
        }
    }
}

