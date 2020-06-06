/*
Based on work by
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class BallTwistPublisher : UnityPublisher<MessageTypes.Geometry.Twist>
    {
        public Rigidbody Ball;

        private MessageTypes.Geometry.Twist message;
        private float previousRealTime;        
        private Vector3 previousPosition = Vector3.zero;

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
            message = new MessageTypes.Geometry.Twist();
            message.linear = new MessageTypes.Geometry.Vector3();
            message.angular = new MessageTypes.Geometry.Vector3();
        }
        private void UpdateMessage()
        {

            Vector3 linearVelocity = Ball.velocity;
            Vector3 position = Ball.position;
                
            message.linear = GetGeometryVector3(linearVelocity.Unity2Ros()); ;
            message.angular = GetGeometryVector3(position.Unity2Ros());

            Publish(message);
        }

        private static MessageTypes.Geometry.Vector3 GetGeometryVector3(Vector3 vector3)
        {
            MessageTypes.Geometry.Vector3 geometryVector3 = new MessageTypes.Geometry.Vector3();
            geometryVector3.x = vector3.x;
            geometryVector3.y = vector3.y;
            geometryVector3.z = vector3.z;
            return geometryVector3;
        }
    }
}
