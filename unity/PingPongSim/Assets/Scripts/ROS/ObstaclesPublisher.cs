/*
Based on work by
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/

using UnityEngine;
using System.Collections.Generic;

namespace RosSharp.RosBridgeClient
{
    public class ObstaclesPublisher : UnityPublisher<MessageTypes.Sensor.PointCloud>
    {
        public List<string> obstaclesTags;
        private MessageTypes.Sensor.PointCloud message;
        private List<GameObject> obstacles;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void Update() {
            obstacles.Clear();
            foreach (string obstaclesTag in obstaclesTags)
            {
                GameObject[] obstaclesArray = GameObject.FindGameObjectsWithTag(obstaclesTag);
                foreach(GameObject obstacle in obstaclesArray)
                {
                    obstacles.Add(obstacle);
                }
            }  
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {
            obstacles = new List<GameObject>();

            message = new MessageTypes.Sensor.PointCloud();
            MessageTypes.Sensor.ChannelFloat32 xWidthChannel = new MessageTypes.Sensor.ChannelFloat32();
            MessageTypes.Sensor.ChannelFloat32 yWidthChannel = new MessageTypes.Sensor.ChannelFloat32();
            MessageTypes.Sensor.ChannelFloat32 HeightChannel = new MessageTypes.Sensor.ChannelFloat32();

            xWidthChannel.name = "xWidths";
            yWidthChannel.name = "yWidths";
            HeightChannel.name = "heights";

            MessageTypes.Sensor.ChannelFloat32[] channels = {xWidthChannel, yWidthChannel, HeightChannel};
            message.channels = channels;
        }
        private void UpdateMessage()
        {
            processObstacles();
            Publish(message);
        }

        private void processObstacles()
        {
            List<MessageTypes.Geometry.Point32> points = new List<MessageTypes.Geometry.Point32>();
            List<float> xWidths = new List<float>();
            List<float> yWidths = new List<float>();
            List<float> heights = new List<float>();
            foreach (GameObject obstacle in obstacles)
            {
                Vector3 position = getObstaclePosition(obstacle);
                position = position.Unity2Ros();
                MessageTypes.Geometry.Point32 point = new MessageTypes.Geometry.Point32(position.x, position.y, position.z);
                points.Add(point);

                Vector3 size = getObstacleDimensions(obstacle);
                size = size.Unity2RosScale();

                xWidths.Add(size.x);
                yWidths.Add(size.y);
                heights.Add(size.z);
            }
            message.points = points.ToArray();
            foreach (MessageTypes.Sensor.ChannelFloat32 channel in message.channels)
            {
                switch(channel.name)
                {
                    case "xWidths":
                        channel.values = xWidths.ToArray();
                        break;
                    case "yWidths":
                        channel.values = yWidths.ToArray();
                        break;
                    case "heights":
                        channel.values = heights.ToArray();
                        break;
                }
            }
        }

        private Vector3 getObstaclePosition(GameObject obstacle)
        {
            BoxCollider collider = obstacle.GetComponent<BoxCollider>();
            return collider.bounds.center;
        }

        private Vector3 getObstacleDimensions(GameObject obstacle)
        {
            BoxCollider collider = obstacle.GetComponent<BoxCollider>();
            return collider.bounds.size;
        }
    }
}
