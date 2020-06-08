using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CubesSpawner : MonoBehaviour
{
    public GameObject originalCube;
    private bool isButtonPressed = false;
    private List<GameObject> cubes;

    void Start()
    {
        cubes = new List<GameObject>();
        originalCube.SetActive(false);
    }

    void Update()
    {
        isButtonPressed = Input.GetKeyDown("o");
    }

    private void FixedUpdate()
    {
        if (isButtonPressed)
        {
            foreach (GameObject obj in cubes)
            {
                GameObject.Destroy(obj);
            }
            cubes.Clear();

            int numOfCubes = (int)Random.Range(3.0f, 5.0f);
            for (int i = 0; i < numOfCubes; i++)
            {
                Vector3 position = new Vector3(Random.Range(0.5f, 2.5f), 2.0f, Random.Range(-1.5f, 1.5f));
                Quaternion orientation = originalCube.transform.rotation;
                GameObject cube = GameObject.Instantiate(originalCube, position, orientation);
                cube.SetActive(true);
                cubes.Add(cube);
            }
        }
    }
}
