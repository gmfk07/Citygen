using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoadSpawner : MonoBehaviour
{
    public GameObject RoadObject;
    public static float MINIMUM_INTERSECTION_DEVIATION = 30;
    public static float ROAD_SNAP_DISTANCE = 50;
    public static float HIGHWAY_POPULATION_SAMPLE_SIZE = 1;
    public static float BRANCH_ANGLE_DEVIATION_FROM_90 = 3;
    public static float STRAIGHT_ANGLE = 15;
    public static float HIGHWAY_BRANCH_POPULATION_THRESHOLD = 0.1f;
    public static float NORMAL_BRANCH_POPULATION_THRESHOLD = 0.1f;
    public static float HIGHWAY_BRANCH_PROBABILITY = 0.02f;
    public static float DEFAULT_BRANCH_PROBABILITY = 0.4f;
    public static int NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY = 10;
    public static float HIGHWAY_SEGMENT_LENGTH = 400;
    public static float DEFAULT_SEGMENT_LENGTH = 300;
    public static int SEGMENT_COUNT_LIMIT = 3;

    // Taken from https://answers.unity.com/questions/421968/normal-distribution-random.html.
    public static float RandomGaussian(float minValue = 0.0f, float maxValue = 1.0f)
    {
        float u, v, S;

        do
        {
            u = 2.0f * UnityEngine.Random.value - 1.0f;
            v = 2.0f * UnityEngine.Random.value - 1.0f;
            S = u * u + v * v;
        }
        while (S >= 1.0f);

        // Standard Normal Distribution
        float std = u * Mathf.Sqrt(-2.0f * Mathf.Log(S) / S);

        // Normal Distribution centered between the min and max value
        // and clamped following the "three-sigma rule"
        float mean = (minValue + maxValue) / 2.0f;
        float sigma = (maxValue - mean) / 3.0f;
        return Mathf.Clamp(std * sigma + mean, minValue, maxValue);
    }

    public static float RandomNear(float center)
    {
        return RandomGaussian(center - center / 3, center + center / 3);
        // TODO: fix this random number gen to make it actually make sense
    }

    public static float RandomStraightAngle()
    {
        return RandomNear(STRAIGHT_ANGLE);
    }

    public static float RandomBranchAngleDeviation()
    {
        return RandomNear(BRANCH_ANGLE_DEVIATION_FROM_90);
    }

    public class Segment
    {
        public Vector3 start;
        public Vector3 end;
        public int t;
        public bool isHighway = false;
        public bool isSevered = false;
        public List<List<Segment>> links = new List<List<Segment>>() {
            new List<Segment>(), new List<Segment>() };
        public System.Action setupBranchLinks = () => { };

        public Segment(Vector3 start, Vector3 end, int t = 0,
            bool isHighway = false, bool isSevered = false)
        {
            this.start = start;
            this.end = end;
            this.t = t;
            this.isHighway = isHighway;
        }

        public Vector3 dir()
        {
            return (end - start).normalized;
        }

        public float length()
        {
            return (end - start).magnitude;
        }

        public List<Segment> linksForEndContaining(Segment segment) {
            if (this.links[0].Contains(segment)) {
                return this.links[0];
            } else if (this.links[1].Contains(segment)) {
                return this.links[1];
            } else {
                return new List<Segment>();
            }
        }

        public void split(Vector3 point, Segment segment, ref List<Segment> segmentList)
        {
            // Split this segment into two parts:
            // one that goes from point to end
            Segment splitPart = new Segment(point, end, t, isHighway);
            segmentList.Add(splitPart);
            // and one that goes from start to point
            this.end = point;

            // The back links from the split part are "this" and
            // the intersecting segment.
            splitPart.links[0] = new List<Segment>() { this, segment };
            // The forward links from the split part are unchanged.
            splitPart.links[1] = this.links[1];
            // The forward links from this part are the split part and
            // intersecting segment.
            this.links[1] = new List<Segment>() { segment, splitPart };

            // Make 'this' and 'splitPart' be forward links of the
            // intersecting segment.
            segment.links[1].Add(this);
            segment.links[1].Add(splitPart);

            // the end of splitPart used to be the end of this segment.
            // go through all linked roads at splitPart's end, and replace their
            // inverse references (this -> splitPart)
            for (int i = 0; i < splitPart.links[1].Count; i++) {
                Segment link = splitPart.links[1][i];
                // Check the backwards links.
                for (int j = 0; j < link.links[0].Count; j++)
                {
                    if (link.links[0][j] == this)
                    {
                        link.links[0][j] = splitPart;
                    }
                }
                // Check forwards links.
                for (int j = 0; j < link.links[1].Count; j++)
                {
                    if (link.links[1][j] == this)
                    {
                        link.links[1][j] = splitPart;
                    }
                }
            }
        }

        public static Segment usingDirection(
            Vector3 start,
            Vector3 dir,
            float length,
            int t,
            bool isHighway = false,
            bool isSevered = false)
        {
            return new Segment(start, new Vector3(
                (start.x + length * dir.x),
                (start.y + length * dir.y),
                (start.z + length * dir.z)
            ),
            t, isHighway, isSevered);
        }

    };

    public float popOnRoad(Segment r)
    {
        return (this.populationAt(r.start.x, r.start.y) +
            this.populationAt(r.end.x, r.end.y)) / 2;
    }

    public float populationAt(float x, float y) {
        return Mathf.PerlinNoise(x, y);
    }

    // Taken from Unity Community Wiki of 3D Math Functions
    // https://wiki.unity3d.com/index.php/3d_Math_functions
    public static bool LineLineIntersection(out Vector3 intersection, Vector3 linePoint1, Vector3 lineVec1, Vector3 linePoint2, Vector3 lineVec2)
    {

        Vector3 lineVec3 = linePoint2 - linePoint1;
        Vector3 crossVec1and2 = Vector3.Cross(lineVec1, lineVec2);
        Vector3 crossVec3and2 = Vector3.Cross(lineVec3, lineVec2);

        float planarFactor = Vector3.Dot(lineVec3, crossVec1and2);

        //is coplanar, and not parrallel
        if (Mathf.Abs(planarFactor) < 0.0001f && crossVec1and2.sqrMagnitude > 0.0001f)
        {
            float s = Vector3.Dot(crossVec3and2, crossVec1and2) / crossVec1and2.sqrMagnitude;
            intersection = linePoint1 + (lineVec1 * s);
            return true;
        }
        else
        {
            intersection = Vector3.zero;
            return false;
        }
    }

    public static Vector3 closestPointOnSegment(Vector3 point, Segment segment)
    {
        Vector3 segmentVector = segment.end - segment.start;
        Vector3 translatedPoint = point - segment.start;
        Vector3 projection = Vector3.Project(segmentVector, translatedPoint);
        if (pointWithinSegment(segment, point))
        {
            return projection;
        }
        if ((segment.start - point).magnitude < (segment.end - point).magnitude)
        {
            return segment.start;
        }
        return segment.end;
    }

    public static bool pointWithinSegment(Segment segment, Vector3 point)
    {
        // Manually exclude the endpoints of the segment.
        // TODO: this isn't very elegant.
        if (point == segment.start || point == segment.end)
        {
            return false;
        }
        float distance = (point - segment.start).magnitude;
        if (distance < segment.length() && (segment.start +
            distance * (segment.end - segment.start).normalized ==
                point))
        {
            return true;
        }
        return false;
    }

    bool localConstraints(Segment segment, ref List<Segment> segments)
    {
        Debug.Log("local constraints");
        // Intersection check.

        // Find the minimum distance for intersections.
        float minDistance = float.PositiveInfinity;
        foreach (Segment other in segments)
        {
            Vector3 intersection = new Vector3();
            LineLineIntersection(out intersection,
                segment.start, segment.end - segment.start,
                other.start, other.end - other.start);
            // Check that this is a real intersection
            float distance = (intersection - segment.start).magnitude;
            if (pointWithinSegment(segment, intersection))
            {
                minDistance = Mathf.Min(minDistance, distance);
            }
        }
        // Find the segment with the minimum distance and do a split.
        foreach (Segment other in segments)
        {
            Vector3 intersection = new Vector3();
            LineLineIntersection(out intersection,
                segment.start, segment.end - segment.start,
                other.start, other.end - other.start);
            // Check that this is a real intersection
            float distance = (intersection - segment.start).magnitude;
            if (pointWithinSegment(segment, intersection))
            {
                if (distance == minDistance)
                {
                    if (Mathf.Abs(Vector3.Angle(segment.dir(), other.dir())) <
                        MINIMUM_INTERSECTION_DEVIATION)
                    {
                        return false;
                    }
                    Debug.Log("found intersection at " + intersection);
                    other.split(intersection, segment, ref segments);
                    segment.end = intersection;
                    segment.isSevered = true;
                    return true;
                }
            }
            // Crossing within radius check.
            if ((segment.end - other.end).magnitude <= ROAD_SNAP_DISTANCE)
            {
                Vector3 point = other.end;
                segment.end = point;
                segment.isSevered = true;
                foreach (Segment seg in other.links[1])
                {
                    if ((seg.start == segment.start && seg.end == segment.end) ||
                        (seg.start == segment.end && seg.end == segment.start))
                    {
                        return false;
                    }
                }

                Debug.Log("crossing within radius");
                foreach (Segment seg in other.links[1])
                {
                    List<Segment> containing = seg.linksForEndContaining(other);
                    containing.Add(segment);
                    segment.links[1].Add(seg);
                }
                other.links[1].Add(segment);
                segment.links[1].Add(other);
                return true;
            }
            // Intersection within radius check.
            Vector3 closestPoint = closestPointOnSegment(segment.end, other);
            if (closestPoint.magnitude < ROAD_SNAP_DISTANCE)
            {

                Debug.Log("road snap distance");
                segment.end = closestPoint;
                segment.isSevered = true;
                // if intersecting lines are too closely aligned don't continue
                if (Vector3.Angle(other.dir(), segment.dir()) <
                    MINIMUM_INTERSECTION_DEVIATION)
                {
                    return false;
                }
                other.split(closestPoint, segment, ref segments);
                return true; 
            }
        }
        return true;
    }

    List<Segment> globalGoals(Segment previousSegment)
    {
        Debug.Log("global goals");
        List<Segment> newBranches = new List<Segment>();
        if (!previousSegment.isSevered)
        {
            // Try continuing straight
            Segment continueStraight = Segment.usingDirection(
                previousSegment.end, previousSegment.dir(),
                previousSegment.length(), 0,
                previousSegment.isHighway,
                previousSegment.isSevered);
            float straightPop = popOnRoad(continueStraight);

            if (previousSegment.isHighway)
            {
                Segment bestSegment = continueStraight;
                float maxPop = straightPop;
                for (int i = 0; i < HIGHWAY_POPULATION_SAMPLE_SIZE; i++)
                {
                    Segment curSegment = Segment.usingDirection(
                        previousSegment.end,
                        Quaternion.AngleAxis(RandomStraightAngle(), Vector3.up) * previousSegment.dir(),
                        previousSegment.length(), 0,
                        previousSegment.isHighway,
                        previousSegment.isSevered);
                    float curPop = popOnRoad(curSegment);
                    if (curPop > maxPop)
                    {
                        bestSegment = curSegment;
                        maxPop = curPop;
                    }
                }
                Debug.Log("adding segment " + bestSegment.start + ", " + bestSegment.end);
                newBranches.Add(bestSegment);
                if (maxPop > HIGHWAY_BRANCH_POPULATION_THRESHOLD)
                {
                    if (new System.Random().NextDouble() < HIGHWAY_BRANCH_PROBABILITY)
                    {
                        newBranches.Add(Segment.usingDirection(
                        previousSegment.end,
                        Quaternion.AngleAxis(-90 + RandomBranchAngleDeviation(), Vector3.up) * previousSegment.dir(),
                        DEFAULT_SEGMENT_LENGTH,
                        previousSegment.isHighway ?
                            NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY : 0));
                    }
                    else
                    {
                        if (new System.Random().NextDouble() < HIGHWAY_BRANCH_PROBABILITY)
                        {
                            newBranches.Add(Segment.usingDirection(
                                previousSegment.end,
                                Quaternion.AngleAxis(90 + RandomBranchAngleDeviation(), Vector3.up) * previousSegment.dir(),
                                DEFAULT_SEGMENT_LENGTH,
                                previousSegment.isHighway ?
                                    NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY : 0));
                        }
                    }
                }
            }
            else if (straightPop > NORMAL_BRANCH_POPULATION_THRESHOLD)
            {
                newBranches.Add(continueStraight);
            }
            if (straightPop > NORMAL_BRANCH_POPULATION_THRESHOLD)
            {
                if (new System.Random().NextDouble() < DEFAULT_BRANCH_PROBABILITY)
                {
                    newBranches.Add(Segment.usingDirection(
                        previousSegment.end,
                        Quaternion.AngleAxis(-90 + RandomBranchAngleDeviation(), Vector3.up) * previousSegment.dir(),
                        DEFAULT_SEGMENT_LENGTH,
                        previousSegment.isHighway ?
                                    NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY : 0));
                }
                else
                {
                    if (new System.Random().NextDouble() < DEFAULT_BRANCH_PROBABILITY)
                    {
                        newBranches.Add(Segment.usingDirection(
                                previousSegment.end,
                                Quaternion.AngleAxis(90 + RandomBranchAngleDeviation(), Vector3.up) * previousSegment.dir(),
                                DEFAULT_SEGMENT_LENGTH,
                                previousSegment.isHighway ?
                                    NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY : 0));
                    }
                }
            }
        }
        foreach (Segment branch in newBranches)
        {
            branch.setupBranchLinks = () =>
            {
                previousSegment.links[1].ForEach((link) =>
                {
                    branch.links[0].Add(link);
                    link.linksForEndContaining(previousSegment).Add(branch);
                });
                previousSegment.links[1].Add(branch);
                branch.links[0].Add(previousSegment);
            };
        }
        Debug.Log("generated new branches = ");
        foreach (Segment branch in newBranches)
        {
            Debug.Log(branch.start + " " + branch.end);
        }
        return newBranches;
    }

    class PriorityQueue<T>
    {
        public List<T> elements = new List<T>() { };
        public Func<T, float> priorityFunction;
        public PriorityQueue(Func<T, float> priorityFunc)
        {
            this.priorityFunction = priorityFunc;
        }
        public void enqueue(T elt)
        {
            elements.Add(elt);
        }
        public T dequeue()
        {
            float minT = float.PositiveInfinity;
            int minT_i = 0;
            for (int i = 0; i < elements.Count; i++)
            {
                float t = this.priorityFunction(elements[i]);
                if (t < minT)
                {
                    minT = t;
                    minT_i = i;
                }
            }
            T elementToDequeue = elements[minT_i];
            elements.RemoveAt(minT_i);
            return elementToDequeue;
        }
        public bool isEmpty()
        {
            return this.elements.Count == 0;
        }
    }

    List<Segment> makeInitialSegments()
    {
        Segment rootSegment = new Segment(
            new Vector3(0, 0, 0),
            new Vector3(HIGHWAY_SEGMENT_LENGTH, 0, 0),
            0, true);
        Segment oppositeDirection = new Segment(
            new Vector3(0, 0, 0),
            new Vector3(-HIGHWAY_SEGMENT_LENGTH, 0, 0),
            0, true);
        rootSegment.links[0].Add(oppositeDirection);
        oppositeDirection.links[0].Add(rootSegment);
        return new List<Segment>() { rootSegment, oppositeDirection };
    }

    void generationStep(ref PriorityQueue<Segment> pq, ref List<Segment> segments)
    {
        Debug.Log("call generationStep");
        Segment minSegment = pq.dequeue();
        Debug.Log("minSegment = " + minSegment.start + " " + minSegment.end);
        bool accepted = localConstraints(minSegment, ref segments);
        Debug.Log("minSegment after localConstraints = " + minSegment.start + " " + minSegment.end);
        Debug.Log("accepted? = " + accepted);
        if (accepted)
        {
            minSegment.setupBranchLinks();
            Debug.Log("minSegment after setupBranchLinks = " + minSegment.start + " " + minSegment.end);
            segments.Add(minSegment);

            Debug.Log("segments after addition of minSegment =");
            foreach (Segment s in segments)
            {
                Debug.Log(s.start + " " + s.end);
            }
            foreach (Segment newSegment in globalGoals(minSegment))
            {
                newSegment.t = minSegment.t + 1 + newSegment.t;
                pq.enqueue(newSegment);
            }
        }
}

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("start");
        PriorityQueue<Segment> pq = new PriorityQueue<Segment>(s => s.t);
        List<Segment> segments = new List<Segment>();

        Debug.Log("initialized pq and segments");

        foreach (Segment initialSegment in makeInitialSegments())
        {
            pq.enqueue(initialSegment);
        }

        Debug.Log("added initial segments to pq");

        while (!pq.isEmpty() && segments.Count < SEGMENT_COUNT_LIMIT)
        {
            generationStep(ref pq, ref segments);
            Debug.Log("new iteration of generation step, segments = ");
            foreach (Segment s in segments)
            {
                Debug.Log(s.start + " " + s.end);
            }
        }

        Debug.Log("segment generation done, segments =");
        foreach (Segment segment in segments)
        {
            Debug.Log(segment.start + " " + segment.end);

            GameObject road = Instantiate(
                RoadObject, Vector3.zero, Quaternion.identity);

            road.transform.localScale = new Vector3(
                0.1f, 0.1f, segment.length());

            road.transform.rotation = Quaternion.FromToRotation(Vector3.forward, segment.end - segment.start);

            road.transform.Translate((segment.start + segment.end) / 2);

        }
        Debug.Break();
    }
}
