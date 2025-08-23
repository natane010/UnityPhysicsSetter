using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;
using Unity.Burst;
using Unity.Jobs;
using System.Collections.Generic;

namespace NatanePhysics.AutomaticDynamicBone.Runtime
{
    /// <summary>
    /// MagicaCloth2のソルバー最適化技術を実装
    /// フレームレート独立の物理計算と並列処理の最適化
    /// </summary>
    [System.Serializable]
    public class ADBSolverOptimization
    {
        [Header("Solver Configuration")]
        [Range(30, 240)]
        [Tooltip("1秒間の物理更新回数")]
        public int solverFrequency = 90;
        
        [Range(1, 8)]
        [Tooltip("サブステップ数（精度向上）")]
        public int substeps = 2;
        
        [Range(0.5f, 2.0f)]
        [Tooltip("タイムスケール")]
        public float timeScale = 1.0f;
        
        [Header("Parallel Processing")]
        [Tooltip("並列処理を有効化")]
        public bool enableParallelProcessing = true;
        
        [Range(16, 256)]
        [Tooltip("バッチサイズ")]
        public int batchSize = 64;
        
        [Header("Optimization")]
        [Tooltip("空間分割を使用")]
        public bool useSpatialPartitioning = true;
        
        [Range(0.1f, 2.0f)]
        [Tooltip("空間分割のセルサイズ")]
        public float spatialCellSize = 0.5f;
        
        [Tooltip("階層的ソルバーを使用")]
        public bool useHierarchicalSolver = true;
        
        [Header("Adaptive Quality")]
        [Tooltip("適応的品質調整を有効化")]
        public bool enableAdaptiveQuality = true;
        
        [Range(30, 144)]
        [Tooltip("目標FPS")]
        public int targetFPS = 60;
        
        [Range(0.5f, 1.0f)]
        [Tooltip("最小品質レベル")]
        public float minQualityLevel = 0.5f;
        
        // 内部状態
        private float accumulatedTime = 0f;
        private float currentQualityLevel = 1.0f;
        private float frameTimeAverage = 0.016f;
        private Queue<float> frameTimes = new Queue<float>();
        private const int FRAME_TIME_SAMPLES = 30;
        
        // 空間分割用
        private NativeParallelMultiHashMap<int3, int> spatialHash;
        private bool spatialHashInitialized = false;
        
        /// <summary>
        /// ソルバーを初期化
        /// </summary>
        public void Initialize(int maxParticles)
        {
            if (useSpatialPartitioning && !spatialHashInitialized)
            {
                int estimatedCells = Mathf.CeilToInt(maxParticles * 1.5f);
                spatialHash = new NativeParallelMultiHashMap<int3, int>(estimatedCells, Allocator.Persistent);
                spatialHashInitialized = true;
            }
        }
        
        /// <summary>
        /// ソルバーを破棄
        /// </summary>
        public void Dispose()
        {
            if (spatialHashInitialized)
            {
                spatialHash.Dispose();
                spatialHashInitialized = false;
            }
        }
        
        /// <summary>
        /// フレームレート独立の更新を実行
        /// </summary>
        public bool ShouldUpdate(float deltaTime, out int updateCount)
        {
            updateCount = 0;
            
            // 適応的品質調整
            if (enableAdaptiveQuality)
            {
                UpdateQualityLevel(deltaTime);
            }
            
            // タイムスケールを適用
            float scaledDeltaTime = deltaTime * timeScale * currentQualityLevel;
            accumulatedTime += scaledDeltaTime;
            
            // ソルバーのタイムステップ
            float solverDeltaTime = 1.0f / solverFrequency;
            
            // 必要な更新回数を計算
            updateCount = 0;
            while (accumulatedTime >= solverDeltaTime)
            {
                accumulatedTime -= solverDeltaTime;
                updateCount++;
                
                // 最大更新回数を制限（フレームスキップ防止）
                if (updateCount >= 4)
                {
                    accumulatedTime = 0f;
                    break;
                }
            }
            
            return updateCount > 0;
        }
        
        /// <summary>
        /// 適応的品質レベルを更新
        /// </summary>
        private void UpdateQualityLevel(float deltaTime)
        {
            // フレーム時間を記録
            frameTimes.Enqueue(deltaTime);
            if (frameTimes.Count > FRAME_TIME_SAMPLES)
            {
                frameTimes.Dequeue();
            }
            
            // 平均フレーム時間を計算
            float sum = 0f;
            foreach (float time in frameTimes)
            {
                sum += time;
            }
            frameTimeAverage = sum / frameTimes.Count;
            
            // FPSを計算
            float currentFPS = 1.0f / frameTimeAverage;
            
            // 品質レベルを調整
            if (currentFPS < targetFPS * 0.9f)
            {
                // パフォーマンスが低い：品質を下げる
                currentQualityLevel = Mathf.Max(currentQualityLevel - 0.01f, minQualityLevel);
            }
            else if (currentFPS > targetFPS * 1.1f)
            {
                // パフォーマンスが高い：品質を上げる
                currentQualityLevel = Mathf.Min(currentQualityLevel + 0.01f, 1.0f);
            }
        }
        
        /// <summary>
        /// 空間ハッシュを更新
        /// </summary>
        public JobHandle UpdateSpatialHash(NativeArray<float3> positions, JobHandle dependency)
        {
            if (!useSpatialPartitioning || !spatialHashInitialized)
                return dependency;
            
            var job = new UpdateSpatialHashJob
            {
                positions = positions,
                spatialHash = spatialHash.AsParallelWriter(),
                cellSize = spatialCellSize
            };
            
            return job.Schedule(positions.Length, batchSize, dependency);
        }
        
        /// <summary>
        /// 階層的ソルバーを実行
        /// </summary>
        public JobHandle ExecuteHierarchicalSolver(
            NativeArray<float3> positions,
            NativeArray<float3> velocities,
            NativeArray<float> masses,
            NativeArray<int> hierarchy,
            float deltaTime,
            JobHandle dependency)
        {
            if (!useHierarchicalSolver)
            {
                // 通常のソルバー
                return ExecuteStandardSolver(positions, velocities, masses, deltaTime, dependency);
            }
            
            // 階層レベルごとに処理
            int maxLevel = GetMaxHierarchyLevel(hierarchy);
            JobHandle handle = dependency;
            
            for (int level = 0; level <= maxLevel; level++)
            {
                var job = new HierarchicalSolverJob
                {
                    positions = positions,
                    velocities = velocities,
                    masses = masses,
                    hierarchy = hierarchy,
                    currentLevel = level,
                    deltaTime = deltaTime / substeps
                };
                
                handle = job.Schedule(positions.Length, batchSize, handle);
            }
            
            return handle;
        }
        
        /// <summary>
        /// 標準ソルバーを実行
        /// </summary>
        private JobHandle ExecuteStandardSolver(
            NativeArray<float3> positions,
            NativeArray<float3> velocities,
            NativeArray<float> masses,
            float deltaTime,
            JobHandle dependency)
        {
            var job = new StandardSolverJob
            {
                positions = positions,
                velocities = velocities,
                masses = masses,
                deltaTime = deltaTime / substeps
            };
            
            JobHandle handle = dependency;
            for (int i = 0; i < substeps; i++)
            {
                handle = job.Schedule(positions.Length, batchSize, handle);
            }
            
            return handle;
        }
        
        /// <summary>
        /// 最大階層レベルを取得
        /// </summary>
        private int GetMaxHierarchyLevel(NativeArray<int> hierarchy)
        {
            int maxLevel = 0;
            for (int i = 0; i < hierarchy.Length; i++)
            {
                maxLevel = math.max(maxLevel, hierarchy[i]);
            }
            return maxLevel;
        }
        
        /// <summary>
        /// 現在の品質レベルを取得
        /// </summary>
        public float GetQualityLevel()
        {
            return currentQualityLevel;
        }
        
        /// <summary>
        /// 現在のFPSを取得
        /// </summary>
        public float GetCurrentFPS()
        {
            return 1.0f / frameTimeAverage;
        }
        
        /// <summary>
        /// 実効ソルバー周波数を取得
        /// </summary>
        public float GetEffectiveSolverFrequency()
        {
            return solverFrequency * currentQualityLevel;
        }
    }
    
    /// <summary>
    /// 空間ハッシュ更新ジョブ
    /// </summary>
    [BurstCompile]
    public struct UpdateSpatialHashJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> positions;
        [WriteOnly] public NativeParallelMultiHashMap<int3, int>.ParallelWriter spatialHash;
        [ReadOnly] public float cellSize;
        
        public void Execute(int index)
        {
            float3 position = positions[index];
            int3 cell = new int3(
                (int)math.floor(position.x / cellSize),
                (int)math.floor(position.y / cellSize),
                (int)math.floor(position.z / cellSize)
            );
            
            spatialHash.Add(cell, index);
        }
    }
    
    /// <summary>
    /// 階層的ソルバージョブ
    /// </summary>
    [BurstCompile]
    public struct HierarchicalSolverJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> velocities;
        [ReadOnly] public NativeArray<float> masses;
        [ReadOnly] public NativeArray<int> hierarchy;
        [ReadOnly] public int currentLevel;
        [ReadOnly] public float deltaTime;
        
        public void Execute(int index)
        {
            if (hierarchy[index] != currentLevel)
                return;
            
            float3 position = positions[index];
            float3 velocity = velocities[index];
            float mass = masses[index];
            
            // 簡単な物理計算（例：重力）
            velocity.y -= 9.81f * deltaTime;
            
            // 速度減衰
            velocity *= math.lerp(1.0f, 0.98f, deltaTime);
            
            // 位置更新
            position += velocity * deltaTime;
            
            positions[index] = position;
            velocities[index] = velocity;
        }
    }
    
    /// <summary>
    /// 標準ソルバージョブ
    /// </summary>
    [BurstCompile]
    public struct StandardSolverJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> velocities;
        [ReadOnly] public NativeArray<float> masses;
        [ReadOnly] public float deltaTime;
        
        public void Execute(int index)
        {
            float3 position = positions[index];
            float3 velocity = velocities[index];
            float mass = masses[index];
            
            // 簡単な物理計算
            velocity.y -= 9.81f * deltaTime;
            velocity *= 0.99f;
            position += velocity * deltaTime;
            
            positions[index] = position;
            velocities[index] = velocity;
        }
    }
}