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
            t = math.dot(planePosition - rayOrigin, planeNormal) / denom;
            return (t >= 0);
        }

        t = default;
        return false;
    }

    struct Ellipse
    {
        public float3 Center;
        public float3 MajorAxis;
        public float3 MinorAxis;
        public float2 AxesLength;
    }

    static float4x4 GetEllipseTransform(in Ellipse ellipse)
    {
        var transform = float4x4.zero;
        transform[0] = new float4(ellipse.MajorAxis * ellipse.AxesLength.x, 0);
        transform[1] = new float4(ellipse.MinorAxis * ellipse.AxesLength.y, 0);
        transform[2] = new float4(math.cross(ellipse.MajorAxis, ellipse.MinorAxis), 0);
        transform[3] = new float4(ellipse.Center, 1);
        return transform;
    }

    static void SetEllipseAxesLengthsFromPointAndAspect(ref Ellipse ellipse, float2 point, float aspect)
    {
        var t = math.atan2(point.y * aspect, point.x);
        ellipse.AxesLength = new float2(point.x / math.cos(t), point.y / math.sin(t));
    }

    static float2 GetNormalAtPoint(in Ellipse ellipse, float2 p)
    {
        var a = ellipse.AxesLength.x;
        var b = ellipse.AxesLength.y;
        var slope = a * a * p.y / (b * b * p.x);
        var normal = math.normalize(new float2(1, slope));
        // Normal points outwards.
        return normal * (p.x >= 0 ? 1 : -1);
    }

    // The point is assumed to lie on the ellipse's plane.
    static float2 GetPointInEllipseCoordinates(in Ellipse ellipse, float3 point)
    {
        var x = math.dot(point - ellipse.Center, ellipse.MajorAxis);
        var y = math.dot(point - ellipse.Center, ellipse.MinorAxis);
        return new float2(x, y);
    }

    static float3 GetPointInWorldCoordinates(in Ellipse ellipse, float2 point)
    {
        return ellipse.Center + ellipse.MajorAxis * point.x + ellipse.MinorAxis * point.y;
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
        // TODO What if // to plane?
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
            var innerEllipse = new Ellipse
            {
                Center = projCenter,
                MajorAxis = majorSemiAxis,
                MinorAxis = minorSemiAxis
            };

            var pointOnInnerEllipse = GetPointInEllipseCoordinates(in innerEllipse, discCenter);
            SetEllipseAxesLengthsFromPointAndAspect(ref innerEllipse, pointOnInnerEllipse, majorOverMinor);

            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(GetPointInWorldCoordinates(in innerEllipse, pointOnInnerEllipse), dotRadius);

            var outerEllipse = innerEllipse;
            // Normal to inner ellipse at disc center.
            var normal = GetNormalAtPoint(in innerEllipse, pointOnInnerEllipse);
            var pointOnOuterEllipse = pointOnInnerEllipse + normal * m_Radius;
            SetEllipseAxesLengthsFromPointAndAspect(ref outerEllipse, pointOnOuterEllipse, majorOverMinor);

            var restore = (float4x4)Handles.matrix;
            // Inner ellipse.
            Handles.matrix = math.mul(restore, GetEllipseTransform(in innerEllipse));
            Handles.color = Color.magenta;
            Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);
            // Outer ellipse.
            Handles.matrix = math.mul(restore, GetEllipseTransform(in outerEllipse));
            Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);

            const int iterations = 8;
            for (var i = 0; i != iterations; ++i)
            {
                var color = Color.yellow;
                color.a = (1 + i) / (float)iterations;
                Handles.color = color;

                normal = GetNormalAtPoint(in innerEllipse, pointOnOuterEllipse);
                pointOnOuterEllipse = pointOnInnerEllipse + normal * m_Radius;
                SetEllipseAxesLengthsFromPointAndAspect(ref outerEllipse, pointOnOuterEllipse, majorOverMinor);

                Handles.matrix = restore;
                Handles.DrawLine(discCenter, GetPointInWorldCoordinates(in outerEllipse, pointOnOuterEllipse));

                Handles.matrix = math.mul(restore, GetEllipseTransform(in outerEllipse));
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