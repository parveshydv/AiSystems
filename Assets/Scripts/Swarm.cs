using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    public float boidForceScale = 20f;

    public float maxSpeed = 5.0f;

    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;

    public float separationWeight = 1.1f;
    public float alignmentWeight = 0.5f;
    public float cohesionWeight = 1f;
    public float goalWeight = 1f;
    public float obstacleWeight = 0.9f;
    public float wanderWeight = 0.3f;

    public float neighbourDistance = 2.0f;

    public float initializationRadius = 1.0f;

    public float initializationForwardRandomRange = 50f;

    private BBoid[] boids;
    private Transform[] boidObjects;

    private float sqrNeighbourDistance;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;

    /// <summary>
    /// Start, this function is called before the first frame
    /// </summary>
    private void Start()
    {
        sqrNeighbourDistance = neighbourDistance * neighbourDistance;
        InitBoids();
    }

    /// <summary>
    /// Initialize the array of boids
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];

        for (int i = 0; i < numberOfBoids; i++)
        {
            Vector3 pos = transform.position + Random.insideUnitSphere * initializationRadius;
            Transform t = Instantiate(boidPrefab, pos, Quaternion.identity);
            boidObjects[i] = t;

            Vector3 randForward = Quaternion.Euler(
                Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange),
                Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange),
                0) * Vector3.forward;

            boids[i].position = pos;
            boids[i].velocity = randForward;
            boids[i].forward = randForward.normalized;
        }
    }

    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        for (int i = 0; i < numberOfBoids; i++)
        {
            boids[i].alignment = Vector3.zero;
            boids[i].cohesion = Vector3.zero;
            boids[i].separation = Vector3.zero;
            boids[i].obstacle = Vector3.zero;
            boids[i].currentTotalForce = Vector3.zero;
        }
    }

    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        if (boids == null) return;
        ResetBoidForces();

        for (int i = 0; i < numberOfBoids; i++)
            ApplyBoidRules(i);

        if (boidZeroNavigatingTowardGoal)
            ApplyGoalRule();

        IntegrateBoids();

        UpdateBoidGraphics();
    }

    private void Update()
    {
        // Render information for boidzero, useful for debugging forces and path planning
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                    Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            { 
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }
        
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment, Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion, Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle, Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
        
    }


    private void ApplyBoidRules(int i)
    {
        Vector3 pos = boids[i].position;
        Vector3 vel = boids[i].velocity;

        Vector3 separation = Vector3.zero;
        Vector3 alignment = Vector3.zero;
        Vector3 cohesion = Vector3.zero;

        int neighbourCount = 0;

        for (int j = 0; j < numberOfBoids; j++)
        {
            if (i == j) continue;

            Vector3 diff = boids[j].position - pos;

            if (diff.sqrMagnitude < sqrNeighbourDistance &&
                Vector3.Dot(boids[i].forward, diff.normalized) > 0)
            {
                neighbourCount++;

                separation += -diff.normalized;
                alignment += boids[j].velocity;
                cohesion += boids[j].position;
            }
        }

        if (neighbourCount == 0)
        {
            Vector3 wander = Random.insideUnitSphere;
            boids[i].currentTotalForce += wanderWeight * ((wander * boidForceScale) - vel);
            return;
        }

        alignment /= neighbourCount;
        cohesion = (cohesion / neighbourCount) - pos;

        Vector3 obstacleForce = ComputeObstacleForce(i) + ComputeWorldBounds(i);

        boids[i].currentTotalForce +=
            separationWeight * ((separation.normalized * boidForceScale) - vel) +
            alignmentWeight * ((alignment.normalized * boidForceScale) - vel) +
            cohesionWeight * ((cohesion.normalized * boidForceScale) - vel) +
            obstacleWeight * ((obstacleForce * boidForceScale) - vel);
        boids[i].alignment = alignment;
        boids[i].cohesion = cohesion;
        boids[i].separation = separation;
        boids[i].obstacle = obstacleForce;
    }

    private Vector3 ComputeObstacleForce(int i)
    {
        Vector3 result = Vector3.zero;
        Vector3 pos = boids[i].position;

        Collider[] detected = Physics.OverlapSphere(pos, obstacleCheckRadius);
        int count = detected.Length;

        for (int k = 0; k < count; k++)
        {
            Vector3 cp = detected[k].ClosestPoint(pos);
            Vector3 dir = pos - cp;
            result += dir.normalized;
        }

        return result;
    }

    private Vector3 ComputeWorldBounds(int i)
    {
        Vector3 p = boids[i].position;
        Vector3 force = Vector3.zero;

        if (p.x > 8) force += Vector3.left;
        if (p.x < -8) force += Vector3.right;
        if (p.z > 8) force += Vector3.back;
        if (p.z < -8) force += Vector3.forward;
        if (p.y > 4) force += Vector3.down;
        if (p.y < 1) force += Vector3.up;

        return force;
    }

    private void IntegrateBoids()
    {
        for (int i = 0; i < numberOfBoids; i++)
        {
            Vector3 vel = boids[i].velocity;
            vel += boids[i].currentTotalForce * Time.fixedDeltaTime;
            vel = Vector3.ClampMagnitude(vel, maxSpeed);

            boids[i].velocity = vel;
            boids[i].position += vel * Time.fixedDeltaTime;
            boids[i].forward = vel.normalized;
        }
    }

    private void UpdateBoidGraphics()
    {
        for (int i = 0; i < numberOfBoids; i++)
        {
            boidObjects[i].position = boids[i].position;
            boidObjects[i].rotation =
                Quaternion.RotateTowards(
                    boidObjects[i].rotation,
                    Quaternion.LookRotation(boids[i].forward),
                    rotationSpeed * Time.deltaTime);
        }
    }

    public void SetGoal(Vector3 goal)
    {
        if (boidZeroNavigatingTowardGoal) return;

        boidZeroGoal = goal;

        NavMeshHit startHit, endHit;

        NavMesh.SamplePosition(boids[0].position, out startHit, 10f, NavMesh.AllAreas);
        NavMesh.SamplePosition(goal, out endHit, 10f, NavMesh.AllAreas);

        boidZeroPath = new NavMeshPath();
        NavMesh.CalculatePath(startHit.position, endHit.position,
            NavMesh.AllAreas, boidZeroPath);

        currentCorner = 0;
        boidZeroNavigatingTowardGoal = true;
    }

    private void ApplyGoalRule()
    {
        if (boidZeroPath == null || boidZeroPath.corners.Length == 0)
        {
        boidZeroNavigatingTowardGoal = false;
        return;
        }

        Vector3 p = boids[0].position;
        Vector3 corner = boidZeroPath.corners[currentCorner];
        Vector3 diff = corner - p;

        if (diff.sqrMagnitude <= 1f)
        {
            currentCorner++;

            if (currentCorner >= boidZeroPath.corners.Length)
            {
                boidZeroNavigatingTowardGoal = false;
                return;
            }

            corner = boidZeroPath.corners[currentCorner];
            diff = corner - p;
            }

        Vector3 desired = diff.normalized * boidForceScale;
        boids[0].currentTotalForce += goalWeight * (desired - boids[0].velocity);
    }

}
