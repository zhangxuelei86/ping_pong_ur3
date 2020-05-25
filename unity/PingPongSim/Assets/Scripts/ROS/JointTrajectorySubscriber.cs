/*
Based on work done by Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class JointTrajectorySubscriber : UnitySubscriber<MessageTypes.Trajectory.JointTrajectory>
    {

        public UR3JointTrajectoryController controller;

        private MessageTypes.Trajectory.JointTrajectory mostRecentMessage;
        private UR3JointTrajectory trajectory = new UR3JointTrajectory();

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
            foreach (MessageTypes.Trajectory.JointTrajectoryPoint point in mostRecentMessage.points) {
                UR3JointTrajectoryPoint ur3_point = new UR3JointTrajectoryPoint();
                for(int i = 0; i < UR3JointTrajectoryPoint.joints_num; i++) {
                    ur3_point.positions.Add(System.Convert.ToSingle(-Mathf.Rad2Deg*point.positions[i]));
                    ur3_point.velocities.Add(System.Convert.ToSingle(-Mathf.Rad2Deg*point.velocities[i]));
                }
                ur3_point.time_from_start = point.time_from_start.nsecs/1000;
                trajectory.points.Add(ur3_point);
            }
            controller.followTrajectory(trajectory);
        }
    }
}

