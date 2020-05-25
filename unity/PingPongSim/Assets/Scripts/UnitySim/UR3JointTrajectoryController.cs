using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UR3JointTrajectoryController : MonoBehaviour
{
    private RobotJointPrismatic _slider;
    private RobotJointRevolute[] _joints;
    private RobotJointRevolute _link1;
    private RobotJointRevolute _link2;
    private RobotJointRevolute _link3;
    private RobotJointRevolute _link4;
    private RobotJointRevolute _link5;
    private RobotJointRevolute _link6;


    private UR3JointTrajectory _traj;
    private UR3JointTrajectoryPoint _next_point;
    private float _start_time;
    private int _traj_index = 0;
    private bool _finished_traj = true;
    private bool _going_to_point = false;

    public float PositionKp = 8.0f, PositionKi = 0.0f, PositionKd = 0.5f;

    void Start()
    {
        _slider = GetComponentInChildren<RobotJointPrismatic>();
        _joints = GetComponentsInChildren<RobotJointRevolute>();
        foreach (RobotJointRevolute joint in _joints) {
            switch(joint.name) {
                case "shoulder_link":
                    _link1 = joint;
                break;
                case "upper_arm_link":
                    _link2 = joint;
                break;
                case "forearm_link":
                    _link3 = joint;
                break;
                case "wrist_1_link":
                    _link4 = joint;
                break;
                case "wrist_2_link":
                    _link5 = joint;
                break;
                case "wrist_3_link":
                    _link6 = joint;
                break;
            }
            joint.tunePosPID(PositionKp,PositionKi,PositionKd);
        }   
    }

    void FixedUpdate()
    {
        if(!_finished_traj) {
            if(!_going_to_point) {
                _next_point = _traj.points[_traj_index];
                linkTrack(_link1, _next_point.positions[0], _next_point.velocities[0]);
                linkTrack(_link2, _next_point.positions[1], _next_point.velocities[1]);
                linkTrack(_link3, _next_point.positions[2], _next_point.velocities[2]);
                linkTrack(_link4, _next_point.positions[3], _next_point.velocities[3]);
                linkTrack(_link5, _next_point.positions[4], _next_point.velocities[4]);
                linkTrack(_link6, _next_point.positions[5], _next_point.velocities[5]);
                _going_to_point = true;
                _traj_index++;
            }
            if(_start_time + _next_point.time_from_start <= Time.fixedTime) {
                if(_traj_index == _traj.points.Count) {
                    _finished_traj = true;
                }
                _going_to_point = false;
            }
        }
    }

    private void linkTrack(RobotJoint link, float position, float velocity) {
        link.setDestination(position, velocity);
    }

    // sets the current joint trajectory to follow
    public bool followTrajectory(UR3JointTrajectory traj) {
        if(traj.points.Count == 0) {
            return false;
        }
        _traj_index = 0;
        _traj = traj;
        _going_to_point = false;
        _finished_traj = false;
        _start_time = Time.fixedTime;
        return true;
    }

    // check if most recent trajectory has been completed
    public bool finishedTrajectory() {
        return _finished_traj;
    }

    // check current state in trajectory
    public int currentPointInTrajectory() {
        return _traj_index;
    }

    // tune postions PID
    public bool tunePositionsPID(float kp, float ki, float kd) {
        if(!_finished_traj) {
            return false;
        }
        PositionKp = kp; PositionKi = ki; PositionKd = kd;
        return true;
    }
}
