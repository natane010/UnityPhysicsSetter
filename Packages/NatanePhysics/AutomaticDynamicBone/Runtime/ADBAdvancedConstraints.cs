using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;
using Unity.Burst;
using System;

namespace NatanePhysics.AutomaticDynamicBone.Runtime
{
    /// <summary>
    /// MagicaCloth2インスパイアの高度な制約システム
    /// 複数の制約タイプを組み合わせて、より自然な物理シミュレーションを実現
    /// </summary>
    [System.Serializable]
    public class ADBAdvancedConstraints
    {
        [Header("Tether Constraint (最大距離制約)")]
        [Range(0f, 1f)]
        [Tooltip("固定点からの最大距離制限の強度")]
        public float tetherStiffness = 0.8f;
        
        [Range(1f, 2f)]
        [Tooltip("最大距離の倍率")]
        public float tetherDistanceScale = 1.2f;
        
        [Header("Angle Constraint (角度制約)")]
        [Range(0f, 1f)]
        [Tooltip("角度復元の強度")]
        public float angleRestoration = 0.5f;
        
        [Range(0f, 180f)]
        [Tooltip("最大角度制限（度）")]
        public float maxAngle = 45f;
        
        [Range(0f, 1f)]
        [Tooltip("角速度の減衰")]
        public float angleVelocityAttenuation = 0.3f;
        
        [Header("Triangle Bending (三角形曲げ制約)")]
        [Range(0f, 1f)]
        [Tooltip("曲げ剛性")]
        public float bendingStiffness = 0.3f;
        
        [Header("Motion Constraint (動き制約)")]
        [Range(0f, 1f)]
        [Tooltip("最大移動距離の制限")]
        public float maxDistanceStiffness = 0.5f;
        
        [Range(0f, 1f)]
        [Tooltip("バックストップの強度")]
        public float backstopStiffness = 0.8f;
        
        [Range(0f, 0.5f)]
        [Tooltip("バックストップ距離")]
        public float backstopDistance = 0.1f;
        
        [Header("Self Collision (自己衝突)")]
        [Tooltip("セルフコリジョンを有効化")]
        public bool enableSelfCollision = false;
        
        [Range(0.01f, 0.5f)]
        [Tooltip("セルフコリジョンの検出半径")]
        public float selfCollisionRadius = 0.05f;
        
        [Range(0f, 1f)]
        [Tooltip("セルフコリジョンの反発力")]
        public float selfCollisionStiffness = 0.5f;
        
        [Header("Solver Settings")]
        [Range(30, 240)]
        [Tooltip("1秒間のソルバー更新回数")]
        public int solverFrequency = 90;
        
        [Range(0f, 1f)]
        [Tooltip("速度減衰率")]
        public float velocityAttenuation = 0.9f;
        
        /// <summary>
        /// Tether制約を適用（最大距離制限）
        /// </summary>
        public float3 ApplyTetherConstraint(float3 currentPosition, float3 fixedPosition, float maxDistance)
        {
            float3 delta = currentPosition - fixedPosition;
            float distance = math.length(delta);
            
            if (distance > maxDistance * tetherDistanceScale)
            {
                float3 direction = delta / math.max(distance, 0.0001f);
                float correction = (distance - maxDistance * tetherDistanceScale) * tetherStiffness;
                return currentPosition - direction * correction;
            }
            
            return currentPosition;
        }
        
        /// <summary>
        /// 角度制約を適用
        /// </summary>
        public quaternion ApplyAngleConstraint(
            quaternion currentRotation, 
            quaternion targetRotation, 
            quaternion previousRotation,
            float deltaTime)
        {
            // 角度差を計算
            float angle = math.degrees(2.0f * math.acos(math.abs(math.dot(currentRotation, targetRotation))));
            
            if (angle > maxAngle)
            {
                // 最大角度を超えている場合は制限
                float t = maxAngle / angle;
                currentRotation = math.slerp(targetRotation, currentRotation, t);
            }
            
            // 角度復元力を適用
            currentRotation = math.slerp(currentRotation, targetRotation, angleRestoration * deltaTime);
            
            // 角速度減衰
            quaternion deltaRotation = math.mul(currentRotation, math.inverse(previousRotation));
            float angleVelocity = math.degrees(2.0f * math.acos(math.abs(deltaRotation.value.w)));
            
            if (angleVelocity > 0.01f)
            {
                float attenuation = 1.0f - angleVelocityAttenuation * deltaTime;
                currentRotation = math.slerp(previousRotation, currentRotation, attenuation);
            }
            
            return currentRotation;
        }
        
        /// <summary>
        /// 三角形曲げ制約を適用
        /// </summary>
        public void ApplyTriangleBendingConstraint(
            ref float3 p0, ref float3 p1, ref float3 p2,
            float3 restP0, float3 restP1, float3 restP2,
            float mass0, float mass1, float mass2)
        {
            // 現在の三角形の法線
            float3 e1 = p1 - p0;
            float3 e2 = p2 - p0;
            float3 currentNormal = math.normalize(math.cross(e1, e2));
            
            // 初期状態の三角形の法線
            float3 restE1 = restP1 - restP0;
            float3 restE2 = restP2 - restP0;
            float3 restNormal = math.normalize(math.cross(restE1, restE2));
            
            // 法線の差分から曲げを計算
            float dot = math.dot(currentNormal, restNormal);
            if (dot < 0.999f)
            {
                float3 axis = math.normalize(math.cross(currentNormal, restNormal));
                float angle = math.acos(math.saturate(dot));
                
                // 曲げ復元力を計算
                float bendingForce = angle * bendingStiffness;
                
                // 質量に基づく重み
                float totalMass = mass0 + mass1 + mass2;
                float w0 = mass0 / totalMass;
                float w1 = mass1 / totalMass;
                float w2 = mass2 / totalMass;
                
                // 各頂点に復元力を適用
                float3 correction = axis * bendingForce;
                p0 += correction * w0;
                p1 += correction * w1;
                p2 += correction * w2;
            }
        }
        
        /// <summary>
        /// モーション制約を適用（最大移動距離とバックストップ）
        /// </summary>
        public float3 ApplyMotionConstraint(
            float3 currentPosition,
            float3 basePosition,
            float3 normal,
            float maxDistance)
        {
            float3 delta = currentPosition - basePosition;
            float distance = math.length(delta);
            
            // 最大距離制限
            if (distance > maxDistance)
            {
                float3 direction = delta / math.max(distance, 0.0001f);
                float correction = (distance - maxDistance) * maxDistanceStiffness;
                currentPosition -= direction * correction;
            }
            
            // バックストップ（法線方向の逆向きへの移動を制限）
            float normalProjection = math.dot(delta, normal);
            if (normalProjection < -backstopDistance)
            {
                float correction = (-backstopDistance - normalProjection) * backstopStiffness;
                currentPosition += normal * correction;
            }
            
            return currentPosition;
        }
        
        /// <summary>
        /// セルフコリジョンを検出して解決
        /// </summary>
        public void ResolveSelfCollision(
            NativeArray<float3> positions,
            NativeArray<float> masses,
            int particleIndex,
            int startIndex,
            int endIndex)
        {
            if (!enableSelfCollision)
                return;
            
            float3 particlePos = positions[particleIndex];
            float particleMass = masses[particleIndex];
            float collisionRadius2 = selfCollisionRadius * selfCollisionRadius;
            
            for (int i = startIndex; i < endIndex; i++)
            {
                if (i == particleIndex)
                    continue;
                
                float3 otherPos = positions[i];
                float3 delta = particlePos - otherPos;
                float distance2 = math.lengthsq(delta);
                
                if (distance2 < collisionRadius2 && distance2 > 0.0001f)
                {
                    float distance = math.sqrt(distance2);
                    float3 direction = delta / distance;
                    
                    // 質量に基づく反発力の分配
                    float otherMass = masses[i];
                    float totalMass = particleMass + otherMass;
                    float w0 = otherMass / totalMass;
                    float w1 = particleMass / totalMass;
                    
                    float penetration = selfCollisionRadius - distance;
                    float3 correction = direction * (penetration * selfCollisionStiffness);
                    
                    positions[particleIndex] = particlePos + correction * w0;
                    positions[i] = otherPos - correction * w1;
                }
            }
        }
        
        /// <summary>
        /// ソルバーのタイムステップを計算
        /// </summary>
        public float GetSolverDeltaTime()
        {
            return 1.0f / solverFrequency;
        }
        
        /// <summary>
        /// 速度減衰を適用
        /// </summary>
        public float3 ApplyVelocityAttenuation(float3 velocity, float deltaTime)
        {
            float attenuation = math.pow(velocityAttenuation, deltaTime * solverFrequency);
            return velocity * attenuation;
        }
    }
    
    /// <summary>
    /// Burst用の高度な制約計算ジョブ
    /// </summary>
    [BurstCompile]
    public struct AdvancedConstraintJob : Unity.Jobs.IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> restPositions;
        [ReadOnly] public NativeArray<float> masses;
        [ReadOnly] public NativeArray<float3> fixedPositions;
        [ReadOnly] public NativeArray<float> maxDistances;
        [ReadOnly] public float tetherStiffness;
        [ReadOnly] public float tetherDistanceScale;
        [ReadOnly] public float deltaTime;
        
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> positions;
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> velocities;
        
        public void Execute(int index)
        {
            float3 position = positions[index];
            float3 velocity = velocities[index];
            float3 fixedPos = fixedPositions[index];
            float maxDist = maxDistances[index];
            
            // Tether制約
            float3 delta = position - fixedPos;
            float distance = math.length(delta);
            
            if (distance > maxDist * tetherDistanceScale)
            {
                float3 direction = delta / math.max(distance, 0.0001f);
                float correction = (distance - maxDist * tetherDistanceScale) * tetherStiffness;
                position -= direction * correction;
                
                // 速度も調整
                velocity -= direction * math.dot(velocity, direction) * 0.5f;
            }
            
            positions[index] = position;
            velocities[index] = velocity;
        }
    }
}