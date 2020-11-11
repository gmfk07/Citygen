using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Road : ScriptableObject
{

    public Vector3 start;
    public Vector3 end;

    public Road(Vector3 start, Vector3 end)
    {
        this.start = start;
        this.end = end;
    }

    public void Awake()
    {
        Debug.DrawLine(start, end);
    }
}