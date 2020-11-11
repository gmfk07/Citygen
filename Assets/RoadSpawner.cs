using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoadSpawner : MonoBehaviour
{
    public GameObject RoadObject;
    public int cycles = 10;

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 1; i < cycles; i++)
        {
            GameObject newObject = Instantiate(RoadObject, new Vector3(i * 2, 0, 0), Quaternion.identity);
            Road newRoad = newObject.GetComponent<Road>();
            newRoad.roadLength = i;
        }
    }
}
