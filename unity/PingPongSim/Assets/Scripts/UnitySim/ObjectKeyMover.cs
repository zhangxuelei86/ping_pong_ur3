using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectKeyMover : MonoBehaviour
{
    public GameObject MovingObject;
    public float speed;
    public float rotateSpeed;
    private bool isLeftKeyPressed = false;
    private bool isRightKeyPressed = false;
    private bool isUpKeyPressed = false;
    private bool isDownKeyPressed = false;
    private bool isWKeyPressed = false;
    private bool isSKeyPressed = false;
    private bool is8KeyPressed = false;
    private bool is2KeyPressed = false;
    private bool is4KeyPressed = false;
    private bool is6KeyPressed = false;
    void Start()
    {

    }

    void Update()
    {
        isLeftKeyPressed = Input.GetKey(KeyCode.LeftArrow);
        isRightKeyPressed = Input.GetKey(KeyCode.RightArrow);
        isUpKeyPressed = Input.GetKey(KeyCode.UpArrow);
        isDownKeyPressed = Input.GetKey(KeyCode.DownArrow);
        isWKeyPressed = Input.GetKey(KeyCode.W);
        isSKeyPressed = Input.GetKey(KeyCode.S);

        is8KeyPressed = Input.GetKey(KeyCode.Keypad8);
        is2KeyPressed = Input.GetKey(KeyCode.Keypad2);
        is4KeyPressed = Input.GetKey(KeyCode.Keypad4);
        is6KeyPressed = Input.GetKey(KeyCode.Keypad6);

        Vector3 position = transform.position;
        
        if (isLeftKeyPressed)
        {
            position.z += speed*Time.deltaTime;
        }
        else if (isRightKeyPressed)
        {
            position.z -= speed*Time.deltaTime;
        }
        if (isUpKeyPressed)
        {
            position.y += speed*Time.deltaTime;
        }
        else if (isDownKeyPressed)
        {
            position.y -= speed*Time.deltaTime;
        }
        if (isWKeyPressed)
        {
            position.x += speed*Time.deltaTime;
        }
        else if (isSKeyPressed)
        {
            position.x -= speed*Time.deltaTime;
        }
        if (is8KeyPressed)
        {
            MovingObject.transform.Rotate(0,0,-rotateSpeed*Time.deltaTime);
        }
        else if (is2KeyPressed)
        {
            MovingObject.transform.Rotate(0,0,rotateSpeed*Time.deltaTime);
        }
        if (is4KeyPressed)
        {
            MovingObject.transform.Rotate(0,-rotateSpeed*Time.deltaTime,0);
        }
        else if (is6KeyPressed)
        {
            MovingObject.transform.Rotate(0,rotateSpeed*Time.deltaTime,0);
        }
        transform.position = position;
    }
}
