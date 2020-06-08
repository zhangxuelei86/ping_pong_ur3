/*
Author: Tony Le (tony@mechatony.com)

Base class for other joint types (Revolute, Prismatic, Continuous)
Keeps track of position, speed, limits, and their relationships. Derived classes (joints) use this position
to control their respective Unity Joints.
*/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraRenderCap : MonoBehaviour
{
    private Camera ImageCamera;
    private float lastUpdate;
    public float RefreshRate;
    void Start()
    {
        ImageCamera = GetComponent(typeof(Camera)) as Camera;
        ImageCamera.enabled = false;
        lastUpdate = Time.time;
    }

    void Update()
    {
        if (Time.time - lastUpdate > 1.0f/RefreshRate)
        {
            ImageCamera.Render();
            lastUpdate = Time.time;
        }
    }
}