using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

[ExecuteAlways]
public class DiscProjection : MonoBehaviour
{
    [SerializeField, Range(0, 1)] float m_Radius;

    static float3 ProjectOnRay(float3 rayOrigin, float3 rayDirection, float3 point)
    {
        return rayOrigin + rayDirection * math.dot(point - rayOrigin, rayDirection);
    }

    bool IntersectPlane(float3 planeNormal, float3 planePosition, float3 rayOrigin, float3 rayDirection, out float t)
    {
        // Assuming vectors are all normalized
        var denom = math.dot(planeNormal, rayDirection);
        if (denom > 1e-6)
        {
            t = math.dot(planePosition - rayOrigin, planeNormal) / denom;
            return t >= 0;
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

    // Set the ellipse's axes lengths based on a known point lying on the ellipse and the aspect of the ellipse.
    static void SetEllipseAxesLengthsFromPointAndAspect(ref Ellipse ellipse, float2 point, float aspect)
    {
        // Special cases: point lies on axes.
        if (math.abs(point.x) < math.EPSILON)
        {
            ellipse.AxesLength = math.abs(new float2(point.y / aspect, point.y));
            return;
        }

        if (math.abs(point.y) < math.EPSILON)
        {
            ellipse.AxesLength = math.abs(new float2(point.x, point.x * aspect));
            return;
        }

        // We do not worry about aspect being zero. We never encounter this case.
        var t = math.atan2(point.y, point.x * aspect);
        ellipse.AxesLength = new float2(point.x / math.cos(t), point.y / math.sin(t));
    }

    // Normal to the ellipse at a point, in local ellipse coordinates.
    // The normal points outwards.
    static float2 GetNormalAtPoint(in Ellipse ellipse, float2 p)
    {
        var a = ellipse.AxesLength.x;
        var b = ellipse.AxesLength.y;
        var denom = b * b * p.x;
        // Special case if x == 0 or if the ellipse's aspect is zero.
        if (math.abs(denom) < math.EPSILON)
        {
            return new float2(0, p.y >= 0 ? 1 : -1);
        }

        var slope = a * a * p.y / (b * b * p.x);
        var normal = math.normalize(new float2(1, slope));
        // Normal points outwards.
        return normal * (p.x >= 0 ? 1 : -1);
    }

    // From world coordinates to local ellipse coordinates.
    // The point is assumed to lie on the ellipse's plane.
    static float2 GetPointInEllipseCoordinates(in Ellipse ellipse, float3 point)
    {
        var x = math.dot(point - ellipse.Center, ellipse.MajorAxis);
        var y = math.dot(point - ellipse.Center, ellipse.MinorAxis);
        return new float2(x, y);
    }

    // From local ellipse coordinates to world coordinates.
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

        // The ellipse aspect, the ratio of its minor axis length over its major axis length, in [0, 1].
        // angle between disc plane and line.
        var aspect = math.abs(math.dot(lineDir, discNormal));

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
        const float large = 1e4f;

        Handles.color = Color.yellow;
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(linePoint, dotRadius);
        Handles.DrawLine(linePoint, linePoint + lineDir);
        Handles.DrawDottedLine(linePoint - lineDir * large, linePoint + lineDir * large, 2);

        Handles.color = Color.cyan;
        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(discCenter, dotRadius);
        Handles.DrawWireDisc(discCenter, discNormal, m_Radius);

        Gizmos.DrawSphere(projCenter, dotRadius);
        Handles.color = Color.cyan;
        Handles.DrawLine(projCenter - majorSemiAxis * large, projCenter + majorSemiAxis * large);
        Handles.DrawLine(projCenter - minorSemiAxis * large, projCenter + minorSemiAxis * large);
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

            // Initialize inner ellipse based on its known eccentricity and the disc center lying on it.
            var pointOnInnerEllipse = GetPointInEllipseCoordinates(in innerEllipse, discCenter);
            SetEllipseAxesLengthsFromPointAndAspect(ref innerEllipse, pointOnInnerEllipse, aspect);

            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(GetPointInWorldCoordinates(in innerEllipse, pointOnInnerEllipse), dotRadius);

            var outerEllipse = innerEllipse;

            // First guess of the outer ellipse, using the normal to the inner ellipse at disc center.
            var normal = GetNormalAtPoint(in innerEllipse, pointOnInnerEllipse);

            // Special case: if the normal is alongside the major axis,
            // the iterative procedure will stay stuck and not converge, so we apply a slight nudge.
            const float epsilon = 1e-6f;
            if (1.0f - math.abs(normal.x) < epsilon)
            {
                var cos = (1 - epsilon) * (normal.x >= 0 ? 1 : -1);
                var sin = math.sin(math.acos(cos)) * (normal.y >= 0 ? 1 : -1);
                normal = new float2(cos, sin);
            }

            var pointOnOuterEllipse = pointOnInnerEllipse + normal * m_Radius;
            SetEllipseAxesLengthsFromPointAndAspect(ref outerEllipse, pointOnOuterEllipse, aspect);

            var restore = (float4x4)Handles.matrix;

            // Inner ellipse.
            Handles.matrix = math.mul(restore, GetEllipseTransform(in innerEllipse));
            Handles.color = Color.grey;
            Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);

            // Outer ellipse.
            Handles.matrix = math.mul(restore, GetEllipseTransform(in outerEllipse));
            Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);

            // Refine guess of the outer ellipse using the normal to the estimated outer ellipse.
            const int iterations = 32;
            Color.RGBToHSV(Color.red, out var startHue, out _, out _);
            Color.RGBToHSV(Color.green, out var endHue, out _, out _);

            for (var i = 0; i != iterations; ++i)
            {
                Handles.color = Color.HSVToRGB(math.lerp(startHue, endHue, (1 + i) / (float)iterations), 1, 1);

                // Update normal based on the outer ellipse and the point lying on it.
                normal = GetNormalAtPoint(in outerEllipse, pointOnOuterEllipse);

                // Update the outer ellipse based on the updated normal.
                pointOnOuterEllipse = pointOnInnerEllipse + normal * m_Radius;
                SetEllipseAxesLengthsFromPointAndAspect(ref outerEllipse, pointOnOuterEllipse, aspect);

                Handles.matrix = restore;
                Handles.DrawLine(discCenter, GetPointInWorldCoordinates(in outerEllipse, pointOnOuterEllipse));

                Handles.matrix = math.mul(restore, GetEllipseTransform(in outerEllipse));
                Handles.DrawWireDisc(Vector3.zero, Vector3.forward, 1);
            }

            Handles.matrix = restore;
        }
    }
}
