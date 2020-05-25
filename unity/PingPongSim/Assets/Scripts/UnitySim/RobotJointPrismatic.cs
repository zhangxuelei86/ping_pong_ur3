using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotJointPrismatic : RobotJoint
{
    public float Gain;
    public float InitialPosition;
    private RosSharp.PrismaticJointLimitsManager _pris_joint_limits;
    private ConfigurableJoint _config_joint;

    void Start()
    {
        _continuous = false;
        _pris_joint_limits = GetComponent(typeof(RosSharp.PrismaticJointLimitsManager)) as RosSharp.PrismaticJointLimitsManager;
        _config_joint = GetComponent(typeof(ConfigurableJoint)) as ConfigurableJoint;
        setLimits(_pris_joint_limits.PositionLimitMin, _pris_joint_limits.PositionLimitMax);
        setGain(Gain);
        setPosition(InitialPosition);
    }

    void FixedUpdate()
    {
        updatePosition();
        Vector3 target_pos = new Vector3(_position, 0.0f, 0.0f);
        _config_joint.targetPosition = target_pos;
    }
}