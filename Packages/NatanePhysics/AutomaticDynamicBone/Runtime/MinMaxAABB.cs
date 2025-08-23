using System;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace TKPhysics
{
    [System.Serializable]
    public struct MinMaxAABB : IEquatable<MinMaxAABB>
    {
        public static MinMaxAABB identity { get { return new MinMaxAABB(); } }

        public float3 Min;

        public float3 Max;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public MinMaxAABB(float3 a, float3 b)
        {
            Min = math.min(a, b);
            Max = math.max(a, b);
        }
        public MinMaxAABB(MinMaxAABB a, MinMaxAABB b)
        {
            Min = math.min(a.Min, b.Min);
            Max = math.max(a.Max, b.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MinMaxAABB CreateFromCenterAndExtents(float3 center, float3 extents)
        {
            return CreateFromCenterAndHalfExtents(center, extents * 0.5f);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MinMaxAABB CreateFromCenterAndHalfExtents(float3 center, float3 halfExtents)
        {
            return new MinMaxAABB(center - halfExtents, center + halfExtents);
        }

        public float3 Extents
        { get { return Max - Min; }
            set {
                HalfExtents = value *0.5f;
            }
        }
            
        public float3 HalfExtents { get { return (Max - Min) * 0.5f; }
            set { 
                Max = Center + math.abs(value);
                Min = Center - math.abs(value);
            }
        }

        public float3 Center
        {
            get { return (Max + Min) * 0.5f; }
            set {
                var distance =  value- Center;
                Max += distance;
                Min += distance;
            }
        }
            

        public bool IsValid => math.all(Min <= Max);

        public float SurfaceArea
        {
            get
            {
                float3 diff = Max - Min;
                return 2 * math.dot(diff, diff.yzx);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(float3 point) => math.all(point >= Min & point <= Max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(MinMaxAABB aabb) => math.all((Min <= aabb.Min) & (Max >= aabb.Max));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Overlaps(MinMaxAABB aabb)
        {
            return math.all(Max >= aabb.Min & Min <= aabb.Max);
        }
        public bool Overlaps(float3 min, float3 max)
        {
            return math.all(Max >=min & Min <= max);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Expand(float signedDistance)
        {
            Min -= signedDistance;
            Max += signedDistance;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Expand(float3 signedDistance)
        {
            Min -= signedDistance;
            Max += signedDistance;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Encapsulate(MinMaxAABB aabb)
        {
            Min = math.min(Min, aabb.Min);
            Max = math.max(Max, aabb.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Encapsulate(float3 point)
        {
            Min = math.min(Min, point);
            Max = math.max(Max, point);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(MinMaxAABB other)
        {
            return Min.Equals(other.Min) && Max.Equals(other.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("MinMaxAABB({0}, {1})", Min, Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MinMaxAABB Transform(RigidTransform transform, MinMaxAABB aabb)
        {
            float3 halfExtentsInA = aabb.HalfExtents;

            float3 x = math.rotate(transform.rot, new float3(halfExtentsInA.x, 0, 0));
            float3 y = math.rotate(transform.rot, new float3(0, halfExtentsInA.y, 0));
            float3 z = math.rotate(transform.rot, new float3(0, 0, halfExtentsInA.z));

            float3 halfExtentsInB = math.abs(x) + math.abs(y) + math.abs(z);
            float3 centerInB = math.transform(transform, aabb.Center);

            return new MinMaxAABB(centerInB - halfExtentsInB, centerInB + halfExtentsInB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MinMaxAABB Transform(float3x3 transform, MinMaxAABB aabb)
        {
            
            float3 t1 = transform.c0.xyz * aabb.Min.xxx;
            float3 t2 = transform.c0.xyz * aabb.Max.xxx;
            bool3 minMask = t1 < t2;
            MinMaxAABB transformed = new MinMaxAABB(math.select(t2, t1, minMask), math.select(t2, t1, !minMask));
            t1 = transform.c1.xyz * aabb.Min.yyy;
            t2 = transform.c1.xyz * aabb.Max.yyy;
            minMask = t1 < t2;
            transformed.Min += math.select(t2, t1, minMask);
            transformed.Max += math.select(t2, t1, !minMask);
            t1 = transform.c2.xyz * aabb.Min.zzz;
            t2 = transform.c2.xyz * aabb.Max.zzz;
            minMask = t1 < t2;
            transformed.Min += math.select(t2, t1, minMask);
            transformed.Max += math.select(t2, t1, !minMask);
            return transformed;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static MinMaxAABB Rotate(quaternion rotation, MinMaxAABB aabb)
        {
            
            float3x3 transform = new float3x3(rotation);
            float3 half = (aabb.Max - aabb. Min) * 0.5f;
            float3 center = aabb.Center; 
            float3 t1 = transform.c0.xyz * half.xxx;
            float3 t2 = transform.c0.xyz * -half.xxx;
            bool3 minMask = t1 < t2;
            MinMaxAABB transformed = new MinMaxAABB(math.select(t2, t1, minMask), math.select(t2, t1, !minMask));
            t1 = transform.c1.xyz * half.yyy;
            t2 = transform.c1.xyz * -half.yyy;
            minMask = t1 < t2;
            transformed.Min += math.select(t2, t1, minMask);
            transformed.Max += math.select(t2, t1, !minMask);
            t1 = transform.c2.xyz * half.zzz;
            t2 = transform.c2.xyz * -half.zzz;
            minMask = t1 < t2;
            transformed.Min += math.select(t2, t1, minMask);
            transformed.Max += math.select(t2, t1, !minMask);

            transformed.Min += center;
            transformed.Max += center;

            return transformed;
        }
    }

}
