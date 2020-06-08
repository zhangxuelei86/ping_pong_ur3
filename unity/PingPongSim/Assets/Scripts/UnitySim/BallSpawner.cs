using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallSpawner : MonoBehaviour
{
    private Rigidbody rb;
    private PPBall ball;
    public float VelocityScale;

    private bool isButtonPressed = false;
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        ball = GetComponent<PPBall>();
    }

    void Update()
    {
        isButtonPressed = Input.GetKeyDown("space");
    }

    private void FixedUpdate()
    {
        if (isButtonPressed)
        {
            rb.position = new Vector3(-1.5f, 1.5f, 0.0f);
            rb.velocity = new Vector3(2.5f*VelocityScale, Random.Range(3.0f, 3.75f)*VelocityScale,
                                        Random.Range(-0.5f, 0.5f)*VelocityScale);
            ball.setState(1);
        }
    }
}
