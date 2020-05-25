/*
This the UR3 Joint Trajectory Joint class (datatype) that simplifies the JointTrajectoryPoint message
*/
using System.Collections.Generic;

public class UR3JointTrajectory
{
    public float start_time;
    public List<UR3JointTrajectoryPoint> points = new List<UR3JointTrajectoryPoint>();
}
