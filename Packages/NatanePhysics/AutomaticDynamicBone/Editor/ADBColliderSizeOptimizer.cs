using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using NatanePhysics.AutomaticDynamicBone.Runtime;
using NatanePhysics;

namespace NatanePhysics.AutomaticDynamicBone.Editor
{
    public class ADBColliderSizeOptimizer : EditorWindow
    {
        private enum OptimizationMode
        {
            Performance,
            Balanced,
            Quality
        }
        
        private OptimizationMode selectedMode = OptimizationMode.Balanced;
        private ADBRuntimeController targetController;
        private bool autoAdjustByBoneSize = true;
        private float boneSizeMultiplier = 1.0f;
        private Vector2 scrollPosition;
        
        private Dictionary<string, float> presetSizes = new Dictionary<string, float>()
        {
            { "Hair", 0.3f },
            { "Skirt", 0.5f },
            { "Chest", 0.4f },
            { "Ribbon", 0.2f },
            { "Accessory", 0.3f },
            { "Cloth", 0.6f }
        };
        
        [MenuItem("Window/ADB Collider Size Optimizer")]
        public static void ShowWindow()
        {
            var window = GetWindow<ADBColliderSizeOptimizer>("Collider Size Optimizer");
            window.minSize = new Vector2(350, 400);
            window.Show();
        }
        
        private void OnGUI()
        {
            scrollPosition = EditorGUILayout.BeginScrollView(scrollPosition);
            
            DrawHeader();
            EditorGUILayout.Space(10);
            
            DrawTargetSelection();
            EditorGUILayout.Space(10);
            
            if (targetController != null)
            {
                DrawOptimizationSettings();
                EditorGUILayout.Space(10);
                
                DrawPresetSizes();
                EditorGUILayout.Space(10);
                
                DrawActions();
                EditorGUILayout.Space(10);
                
                DrawAnalysis();
            }
            
            EditorGUILayout.EndScrollView();
        }
        
        private void DrawHeader()
        {
            EditorGUILayout.LabelField("コライダーサイズ最適化ツール", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "このツールは、物理シミュレーションのパフォーマンスと品質のバランスを" +
                "最適化するためのコライダーサイズを自動調整します。",
                MessageType.Info
            );
        }
        
        private void DrawTargetSelection()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("対象の選択", EditorStyles.boldLabel);
            
            targetController = EditorGUILayout.ObjectField(
                "コントローラー", 
                targetController, 
                typeof(ADBRuntimeController), 
                true
            ) as ADBRuntimeController;
            
            if (targetController != null)
            {
                EditorGUILayout.LabelField($"現在のサイズ: {targetController.colliderSize:F2}");
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawOptimizationSettings()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("最適化設定", EditorStyles.boldLabel);
            
            selectedMode = (OptimizationMode)EditorGUILayout.EnumPopup("最適化モード", selectedMode);
            
            string modeDescription = selectedMode switch
            {
                OptimizationMode.Performance => "パフォーマンス優先: 大きめのコライダーで計算負荷を軽減",
                OptimizationMode.Balanced => "バランス: パフォーマンスと品質の最適なバランス",
                OptimizationMode.Quality => "品質優先: 小さめのコライダーで正確な衝突判定",
                _ => ""
            };
            
            EditorGUILayout.HelpBox(modeDescription, MessageType.None);
            
            EditorGUILayout.Space(5);
            
            autoAdjustByBoneSize = EditorGUILayout.Toggle("ボーンサイズに基づく自動調整", autoAdjustByBoneSize);
            
            if (autoAdjustByBoneSize)
            {
                boneSizeMultiplier = EditorGUILayout.Slider("ボーンサイズ係数", boneSizeMultiplier, 0.5f, 2.0f);
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawPresetSizes()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("プリセットサイズ", EditorStyles.boldLabel);
            
            foreach (var preset in presetSizes)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField(preset.Key, GUILayout.Width(100));
                EditorGUILayout.LabelField(preset.Value.ToString("F2"), GUILayout.Width(50));
                
                if (GUILayout.Button("適用", GUILayout.Width(60)))
                {
                    ApplyPresetSize(preset.Value);
                }
                
                EditorGUILayout.EndHorizontal();
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawActions()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("アクション", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginHorizontal();
            
            if (GUILayout.Button("自動最適化", GUILayout.Height(30)))
            {
                AutoOptimize();
            }
            
            if (GUILayout.Button("リセット", GUILayout.Height(30)))
            {
                ResetToDefault();
            }
            
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.Space(5);
            
            EditorGUILayout.BeginHorizontal();
            
            if (GUILayout.Button("テスト (小)", GUILayout.Height(25)))
            {
                TestSize(0.3f);
            }
            
            if (GUILayout.Button("テスト (中)", GUILayout.Height(25)))
            {
                TestSize(0.5f);
            }
            
            if (GUILayout.Button("テスト (大)", GUILayout.Height(25)))
            {
                TestSize(0.8f);
            }
            
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawAnalysis()
        {
            EditorGUILayout.BeginVertical("box");
            EditorGUILayout.LabelField("分析", EditorStyles.boldLabel);
            
            if (targetController.allChain != null && targetController.allChain.Length > 0)
            {
                int totalPoints = 0;
                int totalConstraints = 0;
                
                foreach (var chain in targetController.allChain)
                {
                    if (chain != null && chain.allPointTransforms != null)
                    {
                        totalPoints += chain.allPointTransforms.Length;
                    }
                }
                
                EditorGUILayout.LabelField($"チェーン数: {targetController.allChain.Length}");
                EditorGUILayout.LabelField($"ポイント数: {totalPoints}");
                EditorGUILayout.LabelField($"コライダー数: {targetController.overlapsColliderList?.Count ?? 0}");
                
                EditorGUILayout.Space(5);
                
                float recommendedSize = CalculateRecommendedSize(totalPoints);
                EditorGUILayout.LabelField($"推奨サイズ: {recommendedSize:F2}");
                
                if (Mathf.Abs(targetController.colliderSize - recommendedSize) > 0.1f)
                {
                    EditorGUILayout.HelpBox(
                        $"現在のサイズ ({targetController.colliderSize:F2}) は推奨値と差があります。" +
                        $"「自動最適化」ボタンで調整することをお勧めします。",
                        MessageType.Warning
                    );
                }
            }
            else
            {
                EditorGUILayout.LabelField("チェーンデータがありません");
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void ApplyPresetSize(float size)
        {
            if (targetController == null) return;
            
            Undo.RecordObject(targetController, "Apply Preset Collider Size");
            targetController.colliderSize = size;
            targetController.RefreshSize();
            EditorUtility.SetDirty(targetController);
        }
        
        private void AutoOptimize()
        {
            if (targetController == null) return;
            
            float baseSize = selectedMode switch
            {
                OptimizationMode.Performance => 0.8f,
                OptimizationMode.Balanced => 0.5f,
                OptimizationMode.Quality => 0.3f,
                _ => 0.5f
            };
            
            if (autoAdjustByBoneSize && targetController.allChain != null)
            {
                float avgBoneDistance = CalculateAverageBoneDistance();
                baseSize *= (avgBoneDistance * boneSizeMultiplier);
                baseSize = Mathf.Clamp(baseSize, 0.1f, 2.0f);
            }
            
            Undo.RecordObject(targetController, "Auto Optimize Collider Size");
            targetController.colliderSize = baseSize;
            targetController.RefreshSize();
            EditorUtility.SetDirty(targetController);
            
            EditorUtility.DisplayDialog(
                "最適化完了",
                $"コライダーサイズを {baseSize:F2} に設定しました。",
                "OK"
            );
        }
        
        private void ResetToDefault()
        {
            if (targetController == null) return;
            
            Undo.RecordObject(targetController, "Reset Collider Size");
            targetController.colliderSize = 0.5f;
            targetController.RefreshSize();
            EditorUtility.SetDirty(targetController);
        }
        
        private void TestSize(float size)
        {
            if (targetController == null) return;
            
            targetController.colliderSize = size;
            targetController.RefreshSize();
            
            if (!Application.isPlaying)
            {
                EditorUtility.DisplayDialog(
                    "テストモード",
                    "再生モードでテストすることをお勧めします。",
                    "OK"
                );
            }
        }
        
        private float CalculateRecommendedSize(int pointCount)
        {
            float baseSize = selectedMode switch
            {
                OptimizationMode.Performance => 0.8f,
                OptimizationMode.Balanced => 0.5f,
                OptimizationMode.Quality => 0.3f,
                _ => 0.5f
            };
            
            // ポイント数に基づく調整
            if (pointCount > 100)
            {
                baseSize *= 1.2f;
            }
            else if (pointCount < 30)
            {
                baseSize *= 0.8f;
            }
            
            return Mathf.Clamp(baseSize, 0.1f, 2.0f);
        }
        
        private float CalculateAverageBoneDistance()
        {
            if (targetController == null || targetController.allChain == null)
                return 1.0f;
            
            float totalDistance = 0f;
            int count = 0;
            
            foreach (var chain in targetController.allChain)
            {
                if (chain != null && chain.allPointTransforms != null && chain.allPointTransforms.Length > 1)
                {
                    for (int i = 1; i < chain.allPointTransforms.Length; i++)
                    {
                        if (chain.allPointTransforms[i] != null && chain.allPointTransforms[i - 1] != null)
                        {
                            float distance = Vector3.Distance(
                                chain.allPointTransforms[i].position,
                                chain.allPointTransforms[i - 1].position
                            );
                            totalDistance += distance;
                            count++;
                        }
                    }
                }
            }
            
            return count > 0 ? totalDistance / count : 1.0f;
        }
    }
}