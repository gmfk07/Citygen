using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Road : MonoBehaviour
{
    public float roadLength;

    // Start is called before the first frame update
    void Start()
    {
        transform.localScale = new Vector3(1, 1, roadLength);
    }
}
