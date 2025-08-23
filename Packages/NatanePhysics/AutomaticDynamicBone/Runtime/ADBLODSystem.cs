using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;

namespace NatanePhysics.AutomaticDynamicBone.Runtime
{
    /// <summary>
    /// MagicaCloth2インスパイアのLODシステム
    /// カメラ距離やパフォーマンスに応じて動的に品質を調整
    /// </summary>
    [System.Serializable]
    public class ADBLODSystem
    {
        public enum LODMode
        {
            Distance,           // 距離ベース
            Performance,        // パフォーマンスベース
            DistanceAndPerformance,  // 両方の組み合わせ
            Manual             // 手動制御
        }
        
        [System.Serializable]
        public class LODLevel
        {
            [Tooltip("LODレベル名")]
            public string name = "LOD";
            
            [Tooltip("このLODが有効になる距離")]
            public float distance = 10f;
            
            [Header("シミュレーション設定")]
            [Range(1, 10)]
            [Tooltip("物理演算の反復回数")]
            public int iterationCount = 4;
            
            [Range(0f, 1f)]
            [Tooltip("シミュレーション頻度（1=毎フレーム、0.5=2フレームに1回）")]
            public float simulationFrequency = 1f;
            
            [Range(0f, 1f)]
            [Tooltip("制約の強度")]
            public float constraintStrength = 1f;
            
            [Tooltip("コリジョン検出を有効化")]
            public bool enableCollision = true;
            
            [Tooltip("セルフコリジョンを有効化")]
            public bool enableSelfCollision = false;
            
            [Header("最適化設定")]
            [Tooltip("この距離でシミュレーションを完全停止")]
            public bool disableSimulation = false;
            
            [Range(0.1f, 1f)]
            [Tooltip("パーティクル数の削減率")]
            public float particleReduction = 1f;
        }
        
        [Header("LOD設定")]
        public LODMode lodMode = LODMode.Distance;
        
        [Tooltip("LODレベルのリスト（距離が近い順）")]
        public List<LODLevel> lodLevels = new List<LODLevel>
        {
            new LODLevel { name = "LOD0 (High)", distance = 5f, iterationCount = 8, simulationFrequency = 1f },
            new LODLevel { name = "LOD1 (Medium)", distance = 15f, iterationCount = 4, simulationFrequency = 0.75f },
            new LODLevel { name = "LOD2 (Low)", distance = 30f, iterationCount = 2, simulationFrequency = 0.5f },
            new LODLevel { name = "LOD3 (Minimal)", distance = 50f, iterationCount = 1, simulationFrequency = 0.25f, enableCollision = false }
        };
        
        [Header("カメラ設定")]
        [Tooltip("参照するカメラ（nullの場合はメインカメラ）")]
        public Camera targetCamera;
        
        [Tooltip("カメラ外でのシミュレーション")]
        public bool simulateWhenInvisible = false;
        
        [Range(0f, 180f)]
        [Tooltip("カメラのFOV外マージン（度）")]
        public float cullingMargin = 10f;
        
        [Header("パフォーマンス設定")]
        [Range(30, 120)]
        [Tooltip("目標FPS")]
        public int targetFPS = 60;
        
        [Range(0.5f, 2f)]
        [Tooltip("FPS低下時の品質調整速度")]
        public float qualityAdjustSpeed = 1f;
        
        [Header("デバッグ")]
        public bool showDebugInfo = false;
        public Color debugColor = Color.cyan;
        
        // 内部状態
        private int currentLODIndex = 0;
        private float currentDistance = 0f;
        private bool isVisible = true;
        private float frameTimeAccumulator = 0f;
        private int frameCount = 0;
        private float currentFPS = 60f;
        private float simulationTimer = 0f;
        private Renderer targetRenderer;
        
        /// <summary>
        /// LODシステムを初期化
        /// </summary>
        public void Initialize(Transform target)
        {
            if (targetCamera == null)
            {
                targetCamera = Camera.main;
            }
            
            targetRenderer = target.GetComponentInChildren<Renderer>();
            
            // LODレベルを距離でソート
            lodLevels.Sort((a, b) => a.distance.CompareTo(b.distance));
        }
        
        /// <summary>
        /// LODシステムを更新し、現在のLODレベルを決定
        /// </summary>
        public LODLevel UpdateLOD(Transform target, float deltaTime)
        {
            if (targetCamera == null) return GetCurrentLODLevel();
            
            // FPSを計算
            UpdateFPS(deltaTime);
            
            // 可視性をチェック
            UpdateVisibility(target);
            
            // 距離を計算
            currentDistance = Vector3.Distance(targetCamera.transform.position, target.position);
            
            // LODレベルを決定
            switch (lodMode)
            {
                case LODMode.Distance:
                    UpdateDistanceLOD();
                    break;
                    
                case LODMode.Performance:
                    UpdatePerformanceLOD();
                    break;
                    
                case LODMode.DistanceAndPerformance:
                    UpdateHybridLOD();
                    break;
            }
            
            return GetCurrentLODLevel();
        }
        
        /// <summary>
        /// シミュレーションを実行すべきかチェック
        /// </summary>
        public bool ShouldSimulate(float deltaTime)
        {
            var currentLOD = GetCurrentLODLevel();
            
            if (currentLOD.disableSimulation)
                return false;
            
            if (!isVisible && !simulateWhenInvisible)
                return false;
            
            // シミュレーション頻度に基づいてスキップ
            simulationTimer += deltaTime;
            float simulationInterval = 1f / (60f * currentLOD.simulationFrequency);
            
            if (simulationTimer >= simulationInterval)
            {
                simulationTimer = 0f;
                return true;
            }
            
            return false;
        }
        
        /// <summary>
        /// 現在のLODレベルを取得
        /// </summary>
        public LODLevel GetCurrentLODLevel()
        {
            if (lodLevels == null || lodLevels.Count == 0)
                return new LODLevel();
            
            currentLODIndex = Mathf.Clamp(currentLODIndex, 0, lodLevels.Count - 1);
            return lodLevels[currentLODIndex];
        }
        
        /// <summary>
        /// 現在のLODインデックスを取得
        /// </summary>
        public int GetCurrentLODIndex()
        {
            return currentLODIndex;
        }
        
        /// <summary>
        /// 手動でLODレベルを設定
        /// </summary>
        public void SetLODLevel(int index)
        {
            if (lodMode != LODMode.Manual) return;
            currentLODIndex = Mathf.Clamp(index, 0, lodLevels.Count - 1);
        }
        
        private void UpdateDistanceLOD()
        {
            for (int i = lodLevels.Count - 1; i >= 0; i--)
            {
                if (currentDistance <= lodLevels[i].distance)
                {
                    currentLODIndex = i;
                }
            }
        }
        
        private void UpdatePerformanceLOD()
        {
            float fpsRatio = currentFPS / targetFPS;
            
            if (fpsRatio < 0.9f) // FPSが目標の90%未満
            {
                // 品質を下げる
                if (currentLODIndex < lodLevels.Count - 1)
                {
                    currentLODIndex++;
                }
            }
            else if (fpsRatio > 1.1f) // FPSが目標の110%以上
            {
                // 品質を上げる
                if (currentLODIndex > 0)
                {
                    currentLODIndex--;
                }
            }
        }
        
        private void UpdateHybridLOD()
        {
            // まず距離ベースでLODを決定
            UpdateDistanceLOD();
            
            // パフォーマンスが悪い場合はさらにLODを下げる
            float fpsRatio = currentFPS / targetFPS;
            if (fpsRatio < 0.8f && currentLODIndex < lodLevels.Count - 1)
            {
                currentLODIndex++;
            }
        }
        
        private void UpdateVisibility(Transform target)
        {
            if (targetRenderer != null)
            {
                isVisible = targetRenderer.isVisible;
            }
            else
            {
                // Frustum cullingの簡易チェック
                Vector3 viewportPoint = targetCamera.WorldToViewportPoint(target.position);
                isVisible = viewportPoint.z > 0 && 
                           viewportPoint.x > -0.1f && viewportPoint.x < 1.1f &&
                           viewportPoint.y > -0.1f && viewportPoint.y < 1.1f;
            }
        }
        
        private void UpdateFPS(float deltaTime)
        {
            frameTimeAccumulator += deltaTime;
            frameCount++;
            
            if (frameTimeAccumulator >= 0.5f) // 0.5秒ごとにFPSを更新
            {
                currentFPS = frameCount / frameTimeAccumulator;
                frameTimeAccumulator = 0f;
                frameCount = 0;
            }
        }
        
        /// <summary>
        /// デバッグ情報を描画
        /// </summary>
        public void DrawDebugInfo(Transform target)
        {
            if (!showDebugInfo) return;
            
            var currentLOD = GetCurrentLODLevel();
            
            // LODレベルの球を描画
            Gizmos.color = debugColor;
            foreach (var lod in lodLevels)
            {
                Gizmos.DrawWireSphere(target.position, lod.distance);
            }
            
            // 現在のLODレベルを強調
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(target.position, currentLOD.distance);
            
            #if UNITY_EDITOR
            // エディタでのデバッグ情報表示
            Vector3 debugPos = target.position + Vector3.up * 2f;
            UnityEditor.Handles.Label(debugPos, 
                $"LOD: {currentLOD.name}\n" +
                $"Distance: {currentDistance:F1}m\n" +
                $"FPS: {currentFPS:F1}\n" +
                $"Visible: {isVisible}");
            #endif
        }
        
        /// <summary>
        /// LOD遷移をスムーズにするためのブレンド係数を計算
        /// </summary>
        public float CalculateLODBlend()
        {
            if (currentLODIndex >= lodLevels.Count - 1)
                return 1f;
            
            var currentLOD = lodLevels[currentLODIndex];
            var nextLOD = lodLevels[currentLODIndex + 1];
            
            float blendRange = nextLOD.distance - currentLOD.distance;
            float blendProgress = (currentDistance - currentLOD.distance) / blendRange;
            
            return Mathf.Clamp01(blendProgress);
        }
    }
}