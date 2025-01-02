using System;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

[ExecuteAlways]
public class DiscProjection : MonoBehaviour
{
    [SerializeField, Range(0, 1)] float m_Radius;

    // TODO Project on RAY!
    static float3 ProjectOnRay(float3 rayOrigin, float3 rayDirection, float3 point)
    {
        return rayOrigin + rayDirection * math.dot(point - rayOrigin, rayDirection);
    }

    bool IntersectPlane(float3 planeNormal, float3 planePosition, float3 rayOrigin, float3 rayDirection, out float t)
    {
        // Assuming vectors are all normalized
        float denom = math.dot(planeNormal, rayDirection);
        if (denom > 1e-6)
        {
            var p0l0 = planePosition - rayOrigin;
            t = math.dot(p0l0, planeNormal) / denom;
            return (t >= 0);
        }

        t = default;
        return false;
    }

    void OnDrawGizmos()
    {
        var discCenter = (float3)transform.position;
        var discNormal = (float3)(transform.rotation * Vector3.forward);

        var linePoint = (float3)Vector3.zero;
        var lineDir = (float3)Vector3.forward;

        if (math.dot(discNormal, lineDir) < 0)
        {
            discNormal *= -1;
        }

        const float dotRadius = 0.04f;

        // angle between disc plane and line.
        var majorOverMinor = 1.0f / math.abs(math.dot(lineDir, discNormal));

        var minorSemiAxis = math.cross(lineDir, transform.forward);
        var majorSemiAxis = math.cross(minorSemiAxis, discNormal);
        minorSemiAxis = math.normalize(minorSemiAxis);
        majorSemiAxis = math.normalize(majorSemiAxis);

        //var projCenter = ProjectOnLine(linePoint, lineDir, discCenter);
        IntersectPlane(discNormal, discCenter, linePoint, lineDir, out var t);
        var projCenter = linePoint + lineDir * t;

        var projOnMinor = ProjectOnRay(projCenter, minorSemiAxis, discCenter);
        var projOnMajor = ProjectOnRay(projCenter, majorSemiAxis, discCenter);

        projOnMinor = ProjectOnRay(projCenter, minorSemiAxis, discCenter);
        projOnMajor = ProjectOnRay(projCenter, majorSemiAxis, discCenter);

        // Expected equal to disc center.
        var onEllipse = projOnMinor + projOnMajor - projCenter;

        Handles.color = Color.yellow;
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(linePoint, dotRadius);
        Handles.DrawLine(linePoint, linePoint + lineDir);

        Handles.color = Color.cyan;
        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(discCenter, dotRadius);
        Handles.DrawWireDisc(discCenter, discNormal, m_Radius);

        Gizmos.DrawSphere(projCenter, dotRadius * 0.5f);
        Handles.color = Color.green;
        Handles.DrawLine(projCenter, projCenter + minorSemiAxis);
        Handles.DrawLine(projCenter, projCenter + majorSemiAxis);
        Handles.DrawDottedLine(projOnMinor, onEllipse, 2);
        Handles.DrawDottedLine(projOnMajor, onEllipse, 2);

        // Ellipse proportions dictated by the angle formed by the disc plane with the line.
        {
            // x, y -> project on axes.
            var x = math.dot(discCenter - projCenter, majorSemiAxis);
            var y = math.dot(discCenter - projCenter, minorSemiAxis);
            var ti = math.atan2(y * majorOverMinor, x);
            var ai = x / math.cos(ti);
            var bi = y / math.sin(ti);

            // Normal to inner ellipse at disc center.
            var normal = math.normalize(new float2(1, ai * ai * y / (bi * bi * x)));
            var p = new float2(x, y) + normal * m_Radius * math.sign(x);
            var p3d = projCenter + majorSemiAxis * p.x + minorSemiAxis * p.y;
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(p3d, dotRadius);

            var to = math.atan2(p.y * majorOverMinor, p.x);
            var ao = p.x / math.cos(to);
            var bo = p.y / math.sin(to);

            var restore = Handles.matrix;
            // Inner ellipse.
            var rotation = Matrix4x4.identity;
            rotation.SetColumn(0, (Vector3)majorSemiAxis * ai);
            rotation.SetColumn(1, (Vector3)minorSemiAxis * bi);
            rotation.SetColumn(2, (Vector3)math.cross(majorSemiAxis, minorSemiAxis));
            rotation.SetColumn(3, (Vector4)new float4(projCenter, 1));
            Handles.matrix *= rotation;
            Handles.color = Color.magenta;
            Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);
            // Outer ellipse.
            rotation.SetColumn(0, (Vector3)majorSemiAxis * ao);
            rotation.SetColumn(1, (Vector3)minorSemiAxis * bo);
            Handles.matrix = restore * rotation;
            Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);

            const int iterations = 8;
            for (var i = 0; i != iterations; ++i)
            {
                var color = Color.yellow;
                color.a = (1 + i) / (float)iterations;
                Handles.color = color;

                normal = math.normalize(new float2(1, ao * ao * p.y / (bo * bo * p.x)));
                p = new float2(x, y) + normal * m_Radius * math.sign(x);
                ;
                to = math.atan2(p.y * majorOverMinor, p.x);
                ao = p.x / math.cos(to);
                bo = p.y / math.sin(to);

                Handles.matrix = restore;
                Handles.DrawLine(discCenter, projCenter + majorSemiAxis * p.x + minorSemiAxis * p.y);

                rotation.SetColumn(0, (Vector3)majorSemiAxis * ao);
                rotation.SetColumn(1, (Vector3)minorSemiAxis * bo);
                Handles.matrix = restore * rotation;
                Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);
            }

            Handles.matrix = restore;
        }
    }

    void OnEnable()
    {
    }

    void OnDisable()
    {
    }
}