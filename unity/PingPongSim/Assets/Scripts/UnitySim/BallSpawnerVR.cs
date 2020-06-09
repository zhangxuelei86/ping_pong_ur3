using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallSpawnerVR : MonoBehaviour
{
    private Rigidbody rb;

    private bool isButtonPressed = false;
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        isButtonPressed = OVRInput.GetDown(OVRInput.RawButton.X);
    }

    private void FixedUpdate()
    {
        if (isButtonPressed)
        {
            rb.position = new Vector3(-1.1f, 1.4f, 0.0f);
            rb.velocity = new Vector3(0.0f, 0.0f, 0.0f);
        }
    }
}
