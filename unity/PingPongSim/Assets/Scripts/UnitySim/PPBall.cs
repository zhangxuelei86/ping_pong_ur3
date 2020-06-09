using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PPBall : MonoBehaviour
{
    private int ballState = 0;
    /*
        0 - INITIAL STATE
        1 - LEFT PLAYER PADDLE - FREE MOTION
        2 - BOUNCED OFF TABLE
        3 - LEFT ROBOT PADDLE - FREE MOTION
    */
    private int maxState = 3;

    private int nextState = 0;
    private float debounceTime = 0.1f;
    private float setTime;
    void Start()
    {

    }

    void FixedUpdate()
    {
        if(nextState != ballState)
        {
            if(Time.fixedTime - setTime > debounceTime)
            {
                ballState = nextState;
            }
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.tag == "player_paddle")
        {
            debounceTime = 0.06f;
            setState(1);
        }
        else if (collision.gameObject.tag == "table")
        {
            debounceTime = 0.02f;
            setState(2);
        }
        else if (collision.gameObject.tag == "robot_paddle")
        {
            debounceTime = 0.06f;
            setState(3);
        }
    }

    public bool setState(int state)
    {
        if (state > maxState || state < 0)
        {
            return false;
        }
        nextState = state;
        setTime = Time.fixedTime;
        return true;
    }

    public int getState()
    {
        return ballState;
    }
}