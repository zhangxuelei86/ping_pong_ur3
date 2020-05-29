using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotJointContinuous : RobotJoint
{
    public float Gain;
    private HingeJoint _hinge_joint;

    void Start()
    {
        _continuous = true;
        _joint_type = "continuous";
        _hinge_joint = GetComponent(typeof(HingeJoint)) as HingeJoint;
        setLimits(-180, 180);
        setGain(Gain);
    }

    void FixedUpdate()
    {
        updatePosition();
        JointSpring spr = _hinge_joint.spring;
        spr.targetPosition = _position;
        _hinge_joint.spring = spr;
    }
}