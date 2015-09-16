using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;

[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class Light2D : MonoBehaviour {

    enum VertexLocation
    {
        Middle = 0,
        Left = -1, 
        Right = 1,
    }

    struct Vertex
    {
        public float Angle;
        public Vector3 Position;
        public VertexLocation Location;
        public bool IsEndpoint;
    }

    [SerializeField]
    private int lightSegments = 8;

    [SerializeField]
    private float lightRadius = 20f;

    [SerializeField]
    private Material lightMaterial;

    private PolygonCollider2D[] colliders;
    private List<Vertex> vertices;

    private Mesh lightMesh;

    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;

    private System.Func<float, float, float> Atan2 = FastMath.Atan2;

    void Awake()
    {
        lightMesh = new Mesh();
        lightMesh.name = string.Format("Light Mesh ({0})", name);
        lightMesh.MarkDynamic();

        meshFilter = GetComponent<MeshFilter>();
        meshFilter.mesh = lightMesh;

        meshRenderer = GetComponent<MeshRenderer>();
        meshRenderer.sharedMaterial = lightMaterial;

        vertices = new List<Vertex>();
    }

    void Update()
    {
        // TODO avoid getting all colliders every frame
        FindLightColliders();
        SetLight();
        RenderLightMesh();
        ResetBounds();
    }

    private void FindLightColliders()
    {
        // TODO filter colliders by LayerMask
        colliders = FindObjectsOfType<PolygonCollider2D>();
    }

    private void SetLight()
    {
        var sortAngles = false;

        // TODO consider using object pool
        vertices.Clear();

        // tolerance range for endpoint recognition
        const float magRange = .15f;

        var colliderVertices = new List<Vertex>();

        // iterate meshes gathering vertices.
        foreach (var collider in colliders)
        {
            colliderVertices.Clear();

            // get polygon raycast vertices
            foreach (var point in collider.points)
            {
                var vertex = new Vertex();

                var worldPoint = collider.transform.TransformPoint(point);
                var distance = (worldPoint - transform.position).magnitude;

                var hit = Physics2D.Raycast(transform.position, worldPoint - transform.position, distance);
                if (hit)
                {
                    vertex.Position = hit.point;

                    vertex.IsEndpoint = Mathf.Abs(worldPoint.sqrMagnitude - hit.point.sqrMagnitude) <= magRange;
                }
                else
                {
                    vertex.Position = worldPoint;
                    vertex.IsEndpoint = true;
                }

                Debug.DrawLine(transform.position, vertex.Position, vertex.IsEndpoint ? Color.red : Color.white);

                // vertex position is saved in light local space.
                vertex.Position = transform.InverseTransformPoint(vertex.Position);
                vertex.Angle = Atan2(vertex.Position.y, vertex.Position.x);


                if (vertex.Position.sqrMagnitude <= lightRadius * lightRadius)
                {
                    colliderVertices.Add(vertex);
                }
            }

            // Identify endpoints
            if (colliderVertices.Count > 0)
            {
                var hiloVertices = new Vertex[2];

                // sort by angle
                colliderVertices.Sort((v1, v2) => v2.Angle.CompareTo(v1.Angle));

                Vertex vertex;
                vertex = colliderVertices[0];
                vertex.Location = VertexLocation.Right;
                hiloVertices[0] = vertex;
                colliderVertices[0] = vertex;

                var lastIndex = colliderVertices.Count - 1;
                vertex = colliderVertices[lastIndex];
                vertex.Location = VertexLocation.Left;
                hiloVertices[1] = vertex;
                colliderVertices[lastIndex] = vertex;

                vertices.AddRange(colliderVertices);


                foreach (var hiloVertex in hiloVertices)
                {
                    if (!hiloVertex.IsEndpoint)
                    {
                        continue;
                    }

                    var position = transform.TransformPoint(hiloVertex.Position);
                    var direction = (position - transform.position).normalized;
                    position += direction * .01f;

                    var hit = Physics2D.Raycast(position, direction, lightRadius);

                    var newVertexPosition = hit ? (Vector3)hit.point
                        : transform.TransformPoint(direction * lightRadius);

                    Debug.DrawLine(position, newVertexPosition, Color.green);

                    vertex = new Vertex();
                    vertex.Position = transform.InverseTransformPoint(newVertexPosition);
                    vertex.Angle = Atan2(newVertexPosition.y, newVertexPosition.x);

                    vertices.Add(vertex);
                }

            }

        }

        var theta = 0;
        //float amount = (Mathf.PI * 2) / lightSegments;
        var amount = 360 / lightSegments;

        // Generate vectors for light cast
        for (int i = 0; i < lightSegments; i++)
        {
            theta = (amount * i) % 360;

            var vertex = new Vertex();
            vertex.Position = new Vector3(FastMath.SinArray[theta], FastMath.CosArray[theta], 0);
            vertex.Angle = Atan2(vertex.Position.y, vertex.Position.x);

            vertex.Position *= lightRadius;
            vertex.Position += transform.position;



            var hit = Physics2D.Raycast(transform.position, vertex.Position - transform.position, lightRadius);
            //Debug.DrawRay(transform.position, v.pos - transform.position, Color.white);

            if (!hit)
            {
                //Debug.DrawLine(transform.position, v.pos, Color.white);
                vertex.Position = transform.InverseTransformPoint(vertex.Position);
                vertices.Add(vertex);
            }
        }

        // sort all vertices by angle
        vertices.Sort((v1, v2) => v2.Angle.CompareTo(v1.Angle));


        var epsilon = 0.00001f;
        for (int i = 0; i < vertices.Count - 1; i++)
        {

            var fstVertex = vertices[i];
            var sndVertex = vertices[i + 1];

            if (Mathf.Abs(fstVertex.Angle - sndVertex.Angle) <= epsilon)
            {

                if (sndVertex.Location == VertexLocation.Right)
                { // Right Ray

                    if (fstVertex.Position.sqrMagnitude > sndVertex.Position.sqrMagnitude)
                    {
                        vertices[i] = sndVertex;
                        vertices[i + 1] = fstVertex;
                    }
                }


                // ALREADY DONE!!
                if (fstVertex.Location == VertexLocation.Left)
                { // Left Ray
                    if (fstVertex.Position.sqrMagnitude < sndVertex.Position.sqrMagnitude)
                    {

                        vertices[i] = sndVertex;
                        vertices[i + 1] = fstVertex;
                    }
                }
            }
        }
    }

    private void RenderLightMesh()
    {
        // fill the mesh with vertices
        var meshVertices = new Vector3[vertices.Count + 1];
        var meshUvs = new Vector2[meshVertices.Length];
        var meshTriangles = new int[(vertices.Count * 3)];

        meshVertices[0] = Vector3.zero;

        // vertices
        for (int i = 0; i < vertices.Count; i++)
        {
            meshVertices[i + 1] = vertices[i].Position;
        }

        // uvs
        for (int i = 0; i < meshVertices.Length; i++)
        {
            meshUvs[i] = meshVertices[i];
        }

        // triangles
        for (int i = 0, j = 0, len = vertices.Count * 3; i < len; i += 3, j++)
        {

            meshTriangles[i] = 0;
            meshTriangles[i + 1] = j + 1;
            
            // last index is 1 if is the last vertex (one loop)
            meshTriangles[i + 2] = (i == len - 3) ? 1 : j + 2;
        }

        lightMesh.Clear();
        lightMesh.vertices = meshVertices;
        lightMesh.uv = meshUvs;
        lightMesh.triangles = meshTriangles;

        // update light material (in case it has changed)
        meshRenderer.sharedMaterial = lightMaterial;
    }

    private void ResetBounds()
    {
        var bounds = lightMesh.bounds;
        bounds.center = Vector3.zero;
        lightMesh.bounds = bounds;
    }

    float pseudoAngle(float dy, float dx)
    {
        // Hight performance for calculate angle on a vector (only for sort)
        // APROXIMATE VALUES -- NOT EXACT!! //
        float ax = Mathf.Abs(dx);
        float ay = Mathf.Abs(dy);
        float p = dy / (ax + ay);
        if (dx < 0)
        {
            p = 2 - p;

        }
        return p;
    }
}
