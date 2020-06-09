/*
Based on work by
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class BallStatePublisher : UnityPublisher<MessageTypes.Std.UInt8>
    {
        public PPBall PingPongBall;

        private MessageTypes.Std.UInt8 message;

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
            message = new MessageTypes.Std.UInt8();
        }
        private void UpdateMessage()
        {
            message.data = System.Convert.ToByte(PingPongBall.getState());
            Publish(message);
        }
    }
}
