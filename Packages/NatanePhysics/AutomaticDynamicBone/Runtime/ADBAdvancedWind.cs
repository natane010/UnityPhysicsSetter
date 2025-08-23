using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;
using Unity.Burst;

namespace NatanePhysics.AutomaticDynamicBone.Runtime
{
    /// <summary>
    /// MagicaCloth2インスパイアの高度な風システム
    /// </summary>
    public enum AdvancedWindMode
    {
        GlobalDirection,    // グローバル方向風
        SphereDirection,   // 球体方向風
        BoxDirection,      // 箱型方向風
        SphereRadial,      // 球体放射風
        Vortex            // 渦巻き風（新規追加）
    }
    
    public enum WindWaveType
    {
        Sine,     // サイン波
        Noise,    // ノイズ波
        Pulse,    // パルス波
        Random    // ランダム波
    }
    
    [System.Serializable]
    public class ADBAdvancedWind : MonoBehaviour
    {
        [Header("風の基本設定")]
        public AdvancedWindMode windMode = AdvancedWindMode.GlobalDirection;
        
        [Range(0f, 30f)]
        [Tooltip("基本風速（m/s）")]
        public float windSpeed = 5f;
        
        [Range(0f, 30f)]
        [Tooltip("最大風速（m/s）")]
        public float maxWindSpeed = 15f;
        
        [Header("風の方向")]
        public Vector3 windDirection = Vector3.forward;
        
        [Header("乱流設定")]
        [Range(0f, 1f)]
        [Tooltip("乱流の強度")]
        public float turbulence = 0.3f;
        
        [Range(0.1f, 10f)]
        [Tooltip("乱流の周波数")]
        public float turbulenceFrequency = 1f;
        
        [Header("波形設定")]
        public WindWaveType waveType = WindWaveType.Sine;
        
        [Range(0.1f, 10f)]
        [Tooltip("波の周波数")]
        public float waveFrequency = 1f;
        
        [Range(0f, 1f)]
        [Tooltip("波の振幅")]
        public float waveAmplitude = 0.5f;
        
        [Header("深度影響")]
        [Range(0f, 1f)]
        [Tooltip("深度による影響の減衰")]
        public float depthInfluence = 0.5f;
        
        [Header("同期設定")]
        [Range(0f, 1f)]
        [Tooltip("風の同期率（0:非同期、1:完全同期）")]
        public float synchronization = 0.5f;
        
        [Header("空間設定")]
        [Tooltip("風の影響範囲（SphereとBoxモード用）")]
        public Bounds windBounds = new Bounds(Vector3.zero, Vector3.one * 10f);
        
        [Tooltip("風の減衰距離")]
        public float falloffDistance = 5f;
        
        // 内部変数
        private float timeOffset;
        private float3 noiseOffset;
        private static readonly int NOISE_SCALE = 10;
        
        private void Start()
        {
            timeOffset = UnityEngine.Random.Range(0f, 100f);
            noiseOffset = new float3(
                UnityEngine.Random.Range(0f, 100f),
                UnityEngine.Random.Range(0f, 100f),
                UnityEngine.Random.Range(0f, 100f)
            );
        }
        
        /// <summary>
        /// 指定位置での風力を計算
        /// </summary>
        public float3 CalculateWindForce(float3 worldPosition, float time, float mass = 1f)
        {
            float3 baseForce = float3.zero;
            
            switch (windMode)
            {
                case AdvancedWindMode.GlobalDirection:
                    baseForce = CalculateGlobalWind(worldPosition, time);
                    break;
                    
                case AdvancedWindMode.SphereDirection:
                    baseForce = CalculateSphereWind(worldPosition, time);
                    break;
                    
                case AdvancedWindMode.BoxDirection:
                    baseForce = CalculateBoxWind(worldPosition, time);
                    break;
                    
                case AdvancedWindMode.SphereRadial:
                    baseForce = CalculateRadialWind(worldPosition, time);
                    break;
                    
                case AdvancedWindMode.Vortex:
                    baseForce = CalculateVortexWind(worldPosition, time);
                    break;
            }
            
            // 乱流を追加
            baseForce += CalculateTurbulence(worldPosition, time);
            
            // 波形を適用
            float waveMultiplier = CalculateWaveMultiplier(time);
            baseForce *= waveMultiplier;
            
            // 質量による影響
            baseForce /= mass;
            
            // 最大風速制限
            float magnitude = math.length(baseForce);
            if (magnitude > maxWindSpeed)
            {
                baseForce = math.normalize(baseForce) * maxWindSpeed;
            }
            
            return baseForce;
        }
        
        private float3 CalculateGlobalWind(float3 worldPosition, float time)
        {
            return math.normalize((float3)windDirection) * windSpeed;
        }
        
        private float3 CalculateSphereWind(float3 worldPosition, float time)
        {
            float3 center = (float3)transform.position;
            float distance = math.distance(worldPosition, center);
            
            if (distance > windBounds.extents.x)
                return float3.zero;
            
            float falloff = CalculateFalloff(distance, windBounds.extents.x);
            return math.normalize((float3)windDirection) * windSpeed * falloff;
        }
        
        private float3 CalculateBoxWind(float3 worldPosition, float time)
        {
            float3 center = (float3)transform.position;
            float3 localPos = worldPosition - center;
            float3 extents = (float3)windBounds.extents;
            
            if (math.any(math.abs(localPos) > extents))
                return float3.zero;
            
            // 箱の端からの距離に基づく減衰
            float3 distances = math.abs(localPos) / extents;
            float falloff = 1f - math.max(distances.x, math.max(distances.y, distances.z));
            
            return math.normalize((float3)windDirection) * windSpeed * falloff;
        }
        
        private float3 CalculateRadialWind(float3 worldPosition, float time)
        {
            float3 center = (float3)transform.position;
            float3 direction = worldPosition - center;
            float distance = math.length(direction);
            
            if (distance > windBounds.extents.x || distance < 0.01f)
                return float3.zero;
            
            direction = math.normalize(direction);
            float falloff = CalculateFalloff(distance, windBounds.extents.x);
            
            return direction * windSpeed * falloff;
        }
        
        private float3 CalculateVortexWind(float3 worldPosition, float time)
        {
            float3 center = (float3)transform.position;
            float3 toCenter = center - worldPosition;
            float distance = math.length(toCenter);
            
            if (distance > windBounds.extents.x || distance < 0.01f)
                return float3.zero;
            
            // 渦巻きの接線方向を計算
            float3 up = math.normalize((float3)transform.up);
            float3 tangent = math.cross(up, math.normalize(toCenter));
            
            // 中心に向かう力と接線方向の力を組み合わせ
            float falloff = CalculateFalloff(distance, windBounds.extents.x);
            float3 vortexForce = tangent * windSpeed * 0.7f + toCenter * windSpeed * 0.3f;
            
            return vortexForce * falloff;
        }
        
        private float3 CalculateTurbulence(float3 worldPosition, float time)
        {
            if (turbulence <= 0) return float3.zero;
            
            float syncTime = math.lerp(time + timeOffset, time, synchronization);
            
            float3 noiseInput = worldPosition * turbulenceFrequency + noiseOffset + syncTime;
            
            float3 turbulenceForce = new float3(
                noise.snoise(noiseInput),
                noise.snoise(noiseInput + 100f),
                noise.snoise(noiseInput + 200f)
            );
            
            return turbulenceForce * turbulence * windSpeed * 0.5f;
        }
        
        private float CalculateWaveMultiplier(float time)
        {
            float syncTime = math.lerp(time + timeOffset, time, synchronization);
            float waveTime = syncTime * waveFrequency;
            
            switch (waveType)
            {
                case WindWaveType.Sine:
                    return 1f + math.sin(waveTime) * waveAmplitude;
                    
                case WindWaveType.Noise:
                    return 1f + noise.snoise(new float2(waveTime, 0)) * waveAmplitude;
                    
                case WindWaveType.Pulse:
                    float pulse = math.sin(waveTime);
                    return 1f + math.max(0, pulse) * waveAmplitude;
                    
                case WindWaveType.Random:
                    return 1f + UnityEngine.Random.Range(-waveAmplitude, waveAmplitude);
                    
                default:
                    return 1f;
            }
        }
        
        private float CalculateFalloff(float distance, float maxDistance)
        {
            if (falloffDistance <= 0) return 1f;
            
            float falloffStart = maxDistance - falloffDistance;
            if (distance <= falloffStart) return 1f;
            
            float falloffProgress = (distance - falloffStart) / falloffDistance;
            return 1f - math.smoothstep(0f, 1f, falloffProgress);
        }
        
        /// <summary>
        /// 深度による風力の減衰を計算
        /// </summary>
        public float CalculateDepthAttenuation(float depth, float maxDepth)
        {
            if (depthInfluence <= 0 || depth <= 0) return 1f;
            
            float normalizedDepth = math.saturate(depth / maxDepth);
            return math.lerp(1f, 1f - depthInfluence, normalizedDepth);
        }
        
        /// <summary>
        /// 複数の風ゾーンを合成
        /// </summary>
        public static float3 CombineWindForces(ADBAdvancedWind[] windZones, float3 worldPosition, float time)
        {
            float3 totalForce = float3.zero;
            float totalWeight = 0f;
            
            foreach (var wind in windZones)
            {
                if (wind == null || !wind.enabled) continue;
                
                float3 force = wind.CalculateWindForce(worldPosition, time);
                float weight = math.length(force);
                
                totalForce += force;
                totalWeight += weight;
            }
            
            return totalForce;
        }
        
        private void OnDrawGizmosSelected()
        {
            Gizmos.color = new Color(0.5f, 0.8f, 1f, 0.3f);
            
            switch (windMode)
            {
                case AdvancedWindMode.SphereDirection:
                case AdvancedWindMode.SphereRadial:
                    Gizmos.DrawWireSphere(transform.position, windBounds.extents.x);
                    break;
                    
                case AdvancedWindMode.BoxDirection:
                    Gizmos.DrawWireCube(transform.position, windBounds.size);
                    break;
                    
                case AdvancedWindMode.Vortex:
                    DrawVortexGizmo();
                    break;
            }
            
            // 風向きを表示
            Gizmos.color = Color.cyan;
            Vector3 center = transform.position;
            Gizmos.DrawLine(center, center + windDirection.normalized * 2f);
        }
        
        private void DrawVortexGizmo()
        {
            int segments = 32;
            float radius = windBounds.extents.x;
            Vector3 center = transform.position;
            
            for (int i = 0; i < segments; i++)
            {
                float angle1 = (float)i / segments * Mathf.PI * 2f;
                float angle2 = (float)(i + 1) / segments * Mathf.PI * 2f;
                
                Vector3 p1 = center + new Vector3(Mathf.Cos(angle1), 0, Mathf.Sin(angle1)) * radius;
                Vector3 p2 = center + new Vector3(Mathf.Cos(angle2), 0, Mathf.Sin(angle2)) * radius;
                
                Gizmos.DrawLine(p1, p2);
            }
        }
    }
    
    /// <summary>
    /// Burst用の風力計算ジョブ
    /// </summary>
    [BurstCompile]
    public struct WindForceCalculationJob : Unity.Jobs.IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> positions;
        [ReadOnly] public float3 windDirection;
        [ReadOnly] public float windSpeed;
        [ReadOnly] public float turbulence;
        [ReadOnly] public float time;
        
        [WriteOnly] public NativeArray<float3> windForces;
        
        public void Execute(int index)
        {
            float3 position = positions[index];
            
            // 基本風力
            float3 baseWind = windDirection * windSpeed;
            
            // 簡易乱流
            float3 turbulenceForce = new float3(
                noise.snoise(new float2(position.x * 0.1f + time, position.y * 0.1f)),
                noise.snoise(new float2(position.y * 0.1f + time, position.z * 0.1f)),
                noise.snoise(new float2(position.z * 0.1f + time, position.x * 0.1f))
            ) * turbulence * windSpeed * 0.5f;
            
            windForces[index] = baseWind + turbulenceForce;
        }
    }
}