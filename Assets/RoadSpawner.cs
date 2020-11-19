using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoadSpawner : MonoBehaviour
{
    public GameObject RoadObject;
    public static float MINIMUM_INTERSECTION_DEVIATION = 30;
    public static float MINIMUM_ROAD_LENGTH = 100;
    public static float ROAD_SNAP_DISTANCE = 50;
    public static float HIGHWAY_POPULATION_SAMPLE_SIZE = 1;
    public static float BRANCH_ANGLE_DEVIATION_FROM_90 = 3;
    public static float STRAIGHT_ANGLE_DEVIATION_FROM_0 = 30;
    public static float HIGHWAY_BRANCH_POPULATION_THRESHOLD = 0.1f;
    public static float NORMAL_BRANCH_POPULATION_THRESHOLD = 0.01f;
    public static float HIGHWAY_BRANCH_PROBABILITY = 0.1f;
    public static float DEFAULT_BRANCH_PROBABILITY = 0.4f;
    public static int NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY = 10;
    public static float HIGHWAY_SEGMENT_LENGTH = 400;
    public static float DEFAULT_SEGMENT_LENGTH = 300;
    public static int SEGMENT_COUNT_LIMIT = 200;

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
        return RandomGaussian(-STRAIGHT_ANGLE_DEVIATION_FROM_0,
            STRAIGHT_ANGLE_DEVIATION_FROM_0);
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
            Segment splitPart = new Segment(start, point, t, isHighway);
            segmentList.Add(splitPart);
            // and one that goes from start to point
            this.start = point;
            // Make sure that "this" has the same endpoint, because it might
            // still be in the Priority Queue.
            // What's getting finalized is the 1st part (start -> point).

            // The foward links from the split part are "this" and
            // the intersecting segment.
            splitPart.links[1] = new List<Segment>() { this, segment };
            // The backward links from the split part are unchanged.
            splitPart.links[0] = new List<Segment>(this.links[0]);
            // The backward links from this part are the split part and
            // intersecting segment.
            this.links[0] = new List<Segment>() { segment, splitPart };

            // Make 'this' and 'splitPart' be forward links of the
            // intersecting segment.
            segment.links[1].Add(this);
            segment.links[1].Add(splitPart);

            // the start of splitPart used to be the start of this segment.
            // go through all linked roads at splitPart's start, and replace 
            // their inverse references (this -> splitPart)
            for (int i = 0; i < splitPart.links[0].Count; i++) {
                Segment link = splitPart.links[0][i];
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
        //Debug.Log("compute closest point on segment for segment " + segment.start + " " + segment.end + ", point = " + point);
        Vector3 segmentVector = segment.end - segment.start;
        Vector3 translatedPoint = point - segment.start;
        Vector3 projection = Vector3.Project(segmentVector, translatedPoint);
        if (pointWithinSegment(segment, point))
        {
            //Debug.Log("closest point is projection = " + projection);
            return projection;
        }
        if ((segment.start - point).magnitude < (segment.end - point).magnitude)
        {
            //Debug.Log("segment start is closer than segment end");
            return segment.start;
        }
        //Debug.Log("segment end is closer than segment start");
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
        if (minDistance < MINIMUM_ROAD_LENGTH)
        {
            return false;
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
                    if ((other.start - intersection).magnitude <
                        MINIMUM_ROAD_LENGTH ||
                        (other.end - intersection).magnitude <
                        MINIMUM_ROAD_LENGTH)
                    {
                        return false;
                    }
                    // Reject if the angle between roads is too small
                    if ((Mathf.Abs(Vector3.Angle(segment.dir(), other.dir())) <
                        MINIMUM_INTERSECTION_DEVIATION) ||
                        // Segments have direction but 'roads' don't, so
                        // check the 180 degree rotation as well
                        (Mathf.Abs(Vector3.Angle(-segment.dir(), other.dir())) <
                        MINIMUM_INTERSECTION_DEVIATION))
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
        }

        // Find minimum distance to another road end.
        minDistance = float.PositiveInfinity;
        foreach (Segment other in segments)
        {
            minDistance = Mathf.Min(minDistance, (other.end - segment.end).magnitude);
        }
        if (minDistance <= ROAD_SNAP_DISTANCE)
        {
            foreach (Segment other in segments)
            {
                // Crossing within radius check.
                if ((segment.end - other.end).magnitude == minDistance)
                {
                    // If this road would become too short after snap, reject
                    if ((other.end - segment.start).magnitude <
                        MINIMUM_ROAD_LENGTH)
                    {
                        return false;
                    }
                    if ((Mathf.Abs(Vector3.Angle(other.end - segment.start, other.dir())) <
                            MINIMUM_INTERSECTION_DEVIATION) ||
                            // Segments have direction but 'roads' don't, so
                            // check the 180 degree rotation as well
                            (Mathf.Abs(Vector3.Angle(segment.start - other.end, other.dir())) <
                            MINIMUM_INTERSECTION_DEVIATION))
                    {
                        return false;
                    }
                    Debug.Log("modifying segment from " + segment.start + " " + segment.end + " " + segment.length());
                    Vector3 point = other.end;
                    segment.end = point;
                    Debug.Log("to " + segment.start + " " + segment.end + " " + segment.length());
                    segment.isSevered = true;
                    foreach (Segment seg in other.links[1])
                    {
                        if ((seg.start == segment.start && seg.end == segment.end) ||
                            (seg.start == segment.end && seg.end == segment.start))
                        {
                            return false;
                        }
                    }

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
            }
        }
        minDistance = float.PositiveInfinity;
        foreach (Segment other in segments)
        {
            minDistance = Mathf.Min(minDistance, closestPointOnSegment(segment.end, other).magnitude);
        }
        if (minDistance <= ROAD_SNAP_DISTANCE)
        {
            foreach (Segment other in segments)
            {
                // Intersection within radius check.
                Vector3 closestPoint = closestPointOnSegment(segment.end, other);
                if ((segment.end - closestPoint).magnitude == minDistance)
                {

                    Debug.Log("road snap distance");
                    segment.end = closestPoint;
                    segment.isSevered = true;
                    // if intersecting lines are too closely aligned don't continue
                    if ((Mathf.Abs(Vector3.Angle(segment.dir(), other.dir())) <
                            MINIMUM_INTERSECTION_DEVIATION) ||
                            (Mathf.Abs(Vector3.Angle(-segment.dir(), other.dir())) <
                            MINIMUM_INTERSECTION_DEVIATION))
                    {
                        return false;
                    }
                    other.split(closestPoint, segment, ref segments);
                    return true;
                }
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
                Debug.Log("normal road goes straight");
                newBranches.Add(continueStraight);
            }
            if (straightPop > NORMAL_BRANCH_POPULATION_THRESHOLD)
            {
                if (new System.Random().NextDouble() < DEFAULT_BRANCH_PROBABILITY)
                {
                    Debug.Log("normal road branches left");
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
                        Debug.Log("normal road branches right");
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
            Debug.Log(branch.start + " " + branch.end + " " + branch.length());
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
        return new List<Segment>() { rootSegment };
    }

    void generationStep(ref PriorityQueue<Segment> pq, ref List<Segment> segments)
    {
        Debug.Log("call generationStep");
        Segment minSegment = pq.dequeue();
        Debug.Log("minSegment = " + minSegment.start + " " + minSegment.end + " " + minSegment.length());
        bool accepted = localConstraints(minSegment, ref segments);
        Debug.Log("minSegment after localConstraints = " + minSegment.start + " " + minSegment.end + " " + minSegment.length());
        //Debug.Log("accepted? = " + accepted);
        if (accepted)
        {
            minSegment.setupBranchLinks();
            //Debug.Log("minSegment after setupBranchLinks = " + minSegment.start + " " + minSegment.end);
            segments.Add(minSegment);

            //Debug.Log("segments after addition of minSegment =");
            //foreach (Segment s in segments)
            //{
            //    Debug.Log(s.start + " " + s.end);
            //}
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
            //Debug.Log("new iteration of generation step, segments = ");
            //foreach (Segment s in segments)
            //{
            //    Debug.Log(s.start + " " + s.end + " " + s.length());
            //}
        }

        if (pq.isEmpty()) {
            Debug.Log("pq is empty");
        }
               
        Debug.Log("segment generation done, segments =");
        foreach (Segment segment in segments)
        {
                Debug.Log(segment.start + " " + segment.end + " " + segment.length());

                GameObject road = Instantiate(
                    RoadObject, Vector3.zero, Quaternion.identity);

                road.transform.localScale = new Vector3(
                    segment.isHighway ? 20f : 10f, 0.1f, segment.length());

                road.transform.Translate((segment.start + segment.end) / 2);
                road.transform.rotation = Quaternion.LookRotation(segment.end - (segment.start + segment.end) / 2);
        }
        if (segments.Count >= SEGMENT_COUNT_LIMIT)
        {
            Debug.Log("segment count reached, segments.Count = " + segments.Count);
        }
        Debug.Break();
    }
}
