/*
Base on work of:
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class BoolPublisher : UnityPublisher<MessageTypes.Std.Bool>
    {
        private MessageTypes.Std.Bool message;
        private RosConnector RosConnector;

        protected override void Start()
        {
            RosConnector = GetComponent(typeof(RosConnector)) as RosConnector;
            base.Start();
            message = new MessageTypes.Std.Bool();
        }

        public void SendMessage(bool value)
        {
            message.data = value;
            if(RosConnector.IsConnected.WaitOne(0))
            {
                Publish(message);
            }
        }
    }
}
