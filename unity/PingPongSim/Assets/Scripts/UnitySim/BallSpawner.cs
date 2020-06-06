using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallSpawner : MonoBehaviour
{
    private Rigidbody rb;
    private PPBall ball;
    public float VelocityScale;

    private float timeDelay = 0.04f;
    private float buttonPressedTime;
    private bool stateSet = false;

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
            rb.velocity = new Vector3(2.5f*VelocityScale, Random.Range(2.5f, 3.5f)*VelocityScale,
                                        Random.Range(-0.5f, 0.5f)*VelocityScale);
            buttonPressedTime = Time.fixedTime;
            stateSet = false;
        }
        if (Time.fixedTime - buttonPressedTime > timeDelay)
        {
            if (!stateSet)
            {
                ball.setState(1); // left player paddle
                stateSet = true;
            }
        }
    }
}
