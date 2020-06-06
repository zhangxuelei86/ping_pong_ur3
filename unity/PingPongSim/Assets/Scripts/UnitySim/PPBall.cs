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
    void Start()
    {

    }

    void Update()
    {

    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.tag == "player_paddle")
        {
            setState(1);
        }
        else if (collision.gameObject.tag == "table")
        {
            setState(2);
        }
        else if (collision.gameObject.tag == "robot_paddle")
        {
            setState(3);
        }
    }

    public bool setState(int state)
    {
        if (state > maxState || state < 0)
        {
            return false;
        }

        ballState = state;
        return true;
    }

    public int getState()
    {
        return ballState;
    }
}
