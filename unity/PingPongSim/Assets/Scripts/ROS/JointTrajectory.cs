/*
This the UR3 Joint Trajectory Joint class (datatype) that simplifies the JointTrajectoryPoint message
*/
using System.Collections.Generic;

namespace Robot
{ 
    public class JointTrajectory
    {
        public float start_time;
        public List<string> joint_names = new List<string>();
        public List<JointTrajectoryPoint> points = new List<JointTrajectoryPoint>();
    }
}
