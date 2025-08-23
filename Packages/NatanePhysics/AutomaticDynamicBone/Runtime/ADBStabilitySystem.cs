using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;
using Unity.Burst;

namespace NatanePhysics.AutomaticDynamicBone.Runtime
{
    /// <summary>
    /// MagicaCloth2から学んだ安定性向上システム
    /// 振動抑制、速度制限、リセット後の安定化などを提供
    /// </summary>
    [System.Serializable]
    public class ADBStabilitySystem
    {
        [Header("速度制限設定")]
        [Range(0.1f, 10f)]
        [Tooltip("最大速度制限（m/s）")]
        public float maxVelocity = 5f;
        
        [Range(0.1f, 5f)]
        [Tooltip("最大角速度制限（rad/s）")]
        public float maxAngularVelocity = 2f;
        
        [Header("振動抑制")]
        [Range(0f, 1f)]
        [Tooltip("振動抑制の強度")]
        public float vibrationDamping = 0.5f;
        
        [Range(0f, 1f)]
        [Tooltip("未来予測係数")]
        public float futurePredictionFactor = 0.3f;
        
        [Header("リセット後の安定化")]
        [Range(0f, 2f)]
        [Tooltip("リセット後の安定化時間（秒）")]
        public float stabilizationTimeAfterReset = 0.5f;
        
        [Header("高速移動対応")]
        [Range(0f, 20f)]
        [Tooltip("高速移動の閾値（m/s）")]
        public float fastMovementThreshold = 10f;
        
        [Range(0f, 1f)]
        [Tooltip("高速移動時の減衰係数")]
        public float fastMovementDamping = 0.8f;
        
        [Header("静的摩擦")]
        [Range(0f, 1f)]
        [Tooltip("静的摩擦係数")]
        public float staticFriction = 0.2f;
        
        [Range(0f, 1f)]
        [Tooltip("動的摩擦係数")]
        public float dynamicFriction = 0.1f;
        
        // 内部状態
        private float currentStabilizationTime = 0f;
        private bool isStabilizing = false;
        private float3 previousPosition;
        private quaternion previousRotation;
        private float currentSpeed = 0f;
        
        /// <summary>
        /// 安定化システムを初期化
        /// </summary>
        public void Initialize(float3 position, quaternion rotation)
        {
            previousPosition = position;
            previousRotation = rotation;
            currentSpeed = 0f;
        }
        
        /// <summary>
        /// リセット後の安定化を開始
        /// </summary>
        public void StartStabilization()
        {
            isStabilizing = true;
            currentStabilizationTime = 0f;
        }
        
        /// <summary>
        /// 安定化システムの更新
        /// </summary>
        public void UpdateStabilization(float deltaTime)
        {
            if (isStabilizing)
            {
                currentStabilizationTime += deltaTime;
                if (currentStabilizationTime >= stabilizationTimeAfterReset)
                {
                    isStabilizing = false;
                    currentStabilizationTime = 0f;
                }
            }
        }
        
        /// <summary>
        /// 速度制限を適用
        /// </summary>
        public float3 ApplyVelocityLimit(float3 velocity)
        {
            float magnitude = math.length(velocity);
            if (magnitude > maxVelocity)
            {
                velocity = math.normalize(velocity) * maxVelocity;
            }
            return velocity;
        }
        
        /// <summary>
        /// 角速度制限を適用
        /// </summary>
        public quaternion ApplyAngularVelocityLimit(quaternion currentRotation, quaternion targetRotation, float deltaTime)
        {
            if (deltaTime <= 0) return currentRotation;
            
            float angle = math.degrees(math.acos(math.dot(currentRotation, targetRotation)));
            float maxAngle = math.degrees(maxAngularVelocity * deltaTime);
            
            if (angle > maxAngle)
            {
                float t = maxAngle / angle;
                return math.slerp(currentRotation, targetRotation, t);
            }
            
            return targetRotation;
        }
        
        /// <summary>
        /// 振動抑制を適用（未来予測アルゴリズム）
        /// </summary>
        public float3 ApplyVibrationDamping(float3 currentPosition, float3 velocity, float deltaTime)
        {
            if (vibrationDamping <= 0) return currentPosition;
            
            // 未来位置を予測
            float3 futurePosition = currentPosition + velocity * deltaTime * futurePredictionFactor;
            
            // 予測位置との差分を計算
            float3 diff = futurePosition - currentPosition;
            float diffMagnitude = math.length(diff);
            
            // 振動と判定される小さな動きを減衰
            if (diffMagnitude < 0.1f)
            {
                currentPosition = math.lerp(currentPosition, futurePosition, 1f - vibrationDamping);
            }
            
            return currentPosition;
        }
        
        /// <summary>
        /// 高速移動時の安定化
        /// </summary>
        public float GetFastMovementDamping(float3 currentPosition, float deltaTime)
        {
            if (deltaTime <= 0) return 1f;
            
            float3 velocity = (currentPosition - previousPosition) / deltaTime;
            currentSpeed = math.length(velocity);
            
            if (currentSpeed > fastMovementThreshold)
            {
                float ratio = (currentSpeed - fastMovementThreshold) / fastMovementThreshold;
                return math.lerp(1f, fastMovementDamping, math.saturate(ratio));
            }
            
            previousPosition = currentPosition;
            return 1f;
        }
        
        /// <summary>
        /// 摩擦力を計算
        /// </summary>
        public float3 CalculateFriction(float3 velocity, float3 normal, bool isStatic)
        {
            float friction = isStatic ? staticFriction : dynamicFriction;
            
            // 法線方向の速度成分を除去
            float3 tangentVelocity = velocity - math.dot(velocity, normal) * normal;
            
            // 摩擦力を適用
            return -tangentVelocity * friction;
        }
        
        /// <summary>
        /// 安定化係数を取得（リセット後の安定化中は減衰を強化）
        /// </summary>
        public float GetStabilizationFactor()
        {
            if (!isStabilizing) return 1f;
            
            float progress = currentStabilizationTime / stabilizationTimeAfterReset;
            return math.lerp(0.3f, 1f, progress);
        }
        
        /// <summary>
        /// 現在の状態をデバッグ表示
        /// </summary>
        public void DrawDebugInfo(Vector3 position)
        {
            if (isStabilizing)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(position, 0.1f * (1f - GetStabilizationFactor()));
            }
            
            if (currentSpeed > fastMovementThreshold)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(position, position + (Vector3)(previousPosition - (float3)position));
            }
        }
    }
    
    /// <summary>
    /// Burstコンパイル用の安定性計算ジョブ
    /// </summary>
    [BurstCompile]
    public struct StabilityCalculationJob : Unity.Jobs.IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> currentPositions;
        [ReadOnly] public NativeArray<float3> velocities;
        [ReadOnly] public float deltaTime;
        [ReadOnly] public float maxVelocity;
        [ReadOnly] public float vibrationDamping;
        [ReadOnly] public float futurePredictionFactor;
        
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> stabilizedPositions;
        
        public void Execute(int index)
        {
            float3 position = currentPositions[index];
            float3 velocity = velocities[index];
            
            // 速度制限
            float magnitude = math.length(velocity);
            if (magnitude > maxVelocity)
            {
                velocity = math.normalize(velocity) * maxVelocity;
            }
            
            // 振動抑制（未来予測）
            float3 futurePosition = position + velocity * deltaTime * futurePredictionFactor;
            float3 diff = futurePosition - position;
            float diffMagnitude = math.length(diff);
            
            if (diffMagnitude < 0.1f)
            {
                position = math.lerp(position, futurePosition, 1f - vibrationDamping);
            }
            
            stabilizedPositions[index] = position;
        }
    }
}