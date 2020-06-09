/*
Based on work by
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class TrajectoryStatusPublisher : UnityPublisher<MessageTypes.Std.UInt32>
    {
        public Robot.JointTrajectoryController controller;

        private MessageTypes.Std.UInt32 message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Std.UInt32();
        }
        private void UpdateMessage()
        {
            message.data = (uint)controller.currentPointInTrajectory();
            Publish(message);
        }
    }
}
