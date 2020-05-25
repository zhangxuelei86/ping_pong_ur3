/*
This the UR3 Joint Trajectory Joint class (datatype) that simplifies the JointTrajectoryPoint message
*/
using System.Collections.Generic;

public class UR3JointTrajectoryPoint
{
    public static int joints_num = 6;
    
    public List<float> positions = new List<float>(joints_num);
    public List<float> velocities = new List<float>(joints_num);
    public float time_from_start;  // seconds
}
