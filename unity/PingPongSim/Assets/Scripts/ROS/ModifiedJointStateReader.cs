/*
Based on work by Dr. Martin Bischoff (martin.bischoff@siemens.com)
*/

using RosSharp.Urdf;
using UnityEngine;
using Joint = UnityEngine.Joint;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(Joint)), RequireComponent(typeof(UrdfJoint))]
    public class ModifiedJointStateReader : MonoBehaviour
    {
        private UrdfJoint urdfJoint;
        private RobotJoint _robot_joint;

        private void Start()
        {
            urdfJoint = GetComponent<UrdfJoint>();
            _robot_joint = GetComponent(typeof(RobotJoint)) as RobotJoint;
        }

        public void Read(out string name, out float position, out float velocity, out float effort)
        {
            name = urdfJoint.JointName;
            position = urdfJoint.GetPosition() + _robot_joint.offset*Mathf.Deg2Rad;
            velocity = urdfJoint.GetVelocity();
            effort = urdfJoint.GetEffort();
        }
    }
}
