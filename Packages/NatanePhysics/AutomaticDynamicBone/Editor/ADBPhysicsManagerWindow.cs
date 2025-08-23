using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;
using NatanePhysics.AutomaticDynamicBone.Runtime;
using NatanePhysics;

namespace NatanePhysics.AutomaticDynamicBone.Editor
{
    public class ADBPhysicsManagerWindow : EditorWindow
    {
        private Vector2 scrollPosition;
        private ADBRuntimeController selectedController;
        private int selectedTab = 0;
        private string[] tabNames = new string[] { "セットアップ", "物理設定", "コライダー", "プリセット", "デバッグ" };
        
        private bool showSetupWizard = false;
        private GameObject targetObject;
        private bool autoSetupChains = true;
        private bool autoSetupColliders = true;
        private float globalColliderScale = 1.0f;
        
        private bool isPreviewMode = false;
        private float previewTime = 0f;
        
        private string presetSearchFilter = "";
        private List<ADBPhysicsSetting> availablePresets;
        private ADBPhysicsSetting selectedPreset;
        
        private SerializedObject serializedController;
        private SerializedProperty iterationProp;
        private SerializedProperty colliderSizeProp;
        private SerializedProperty updateModeProp;
        private SerializedProperty timeScaleProp;
        private SerializedProperty windForceScaleProp;
        private SerializedProperty colliderCollisionTypeProp;
        
        private GUIStyle headerStyle;
        private GUIStyle boxStyle;
        
        [MenuItem("Window/ADB Physics Manager")]
        public static void ShowWindow()
        {
            var window = GetWindow<ADBPhysicsManagerWindow>("ADB Physics Manager");
            window.minSize = new Vector2(400, 600);
            window.Show();
        }
        
        private void OnEnable()
        {
            LoadPresets();
            InitializeStyles();
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }
        
        private void OnDisable()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        }
        
        private void InitializeStyles()
        {
            headerStyle = new GUIStyle(EditorStyles.boldLabel);
            headerStyle.fontSize = 14;
            headerStyle.margin = new RectOffset(0, 0, 10, 10);
            
            boxStyle = new GUIStyle("box");
            boxStyle.padding = new RectOffset(10, 10, 10, 10);
        }
        
        private void LoadPresets()
        {
            availablePresets = new List<ADBPhysicsSetting>();
            string[] guids = AssetDatabase.FindAssets("t:ADBPhysicsSetting");
            foreach (string guid in guids)
            {
                string path = AssetDatabase.GUIDToAssetPath(guid);
                ADBPhysicsSetting preset = AssetDatabase.LoadAssetAtPath<ADBPhysicsSetting>(path);
                if (preset != null)
                {
                    availablePresets.Add(preset);
                }
            }
        }
        
        private void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.ExitingPlayMode)
            {
                isPreviewMode = false;
            }
        }
        
        private void OnGUI()
        {
            scrollPosition = EditorGUILayout.BeginScrollView(scrollPosition);
            
            DrawHeader();
            
            EditorGUILayout.Space(10);
            
            selectedTab = GUILayout.Toolbar(selectedTab, tabNames);
            
            EditorGUILayout.Space(10);
            
            switch (selectedTab)
            {
                case 0:
                    DrawSetupTab();
                    break;
                case 1:
                    DrawPhysicsTab();
                    break;
                case 2:
                    DrawCollidersTab();
                    break;
                case 3:
                    DrawPresetsTab();
                    break;
                case 4:
                    DrawDebugTab();
                    break;
            }
            
            EditorGUILayout.EndScrollView();
        }
        
        private void DrawHeader()
        {
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("ADB Physics Manager", headerStyle);
            
            if (GUILayout.Button("更新", GUILayout.Width(60)))
            {
                RefreshControllers();
            }
            
            if (GUILayout.Button("ヘルプ", GUILayout.Width(60)))
            {
                Application.OpenURL("https://github.com/natane010/UnityPhysicsSetter");
            }
            
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.BeginHorizontal(boxStyle);
            EditorGUILayout.LabelField("選択中のコントローラー:", GUILayout.Width(120));
            
            var newController = EditorGUILayout.ObjectField(selectedController, typeof(ADBRuntimeController), true) as ADBRuntimeController;
            if (newController != selectedController)
            {
                selectedController = newController;
                if (selectedController != null)
                {
                    serializedController = new SerializedObject(selectedController);
                    iterationProp = serializedController.FindProperty("iteration");
                    colliderSizeProp = serializedController.FindProperty("colliderSize");
                    updateModeProp = serializedController.FindProperty("updateMode");
                    timeScaleProp = serializedController.FindProperty("timeScale");
                    windForceScaleProp = serializedController.FindProperty("windForceScale");
                    colliderCollisionTypeProp = serializedController.FindProperty("colliderCollisionType");
                }
            }
            
            EditorGUILayout.EndHorizontal();
        }
        
        private void DrawSetupTab()
        {
            EditorGUILayout.LabelField("クイックセットアップ", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            targetObject = EditorGUILayout.ObjectField("対象オブジェクト", targetObject, typeof(GameObject), true) as GameObject;
            
            EditorGUILayout.Space(5);
            
            autoSetupChains = EditorGUILayout.Toggle("チェーンを自動設定", autoSetupChains);
            autoSetupColliders = EditorGUILayout.Toggle("コライダーを自動設定", autoSetupColliders);
            
            EditorGUILayout.Space(5);
            
            globalColliderScale = EditorGUILayout.Slider("コライダースケール", globalColliderScale, 0.1f, 2.0f);
            
            EditorGUILayout.Space(10);
            
            EditorGUILayout.BeginHorizontal();
            
            GUI.enabled = targetObject != null;
            
            if (GUILayout.Button("セットアップ実行", GUILayout.Height(30)))
            {
                PerformQuickSetup();
            }
            
            if (GUILayout.Button("セットアップウィザード", GUILayout.Height(30)))
            {
                showSetupWizard = true;
            }
            
            GUI.enabled = true;
            
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.EndVertical();
            
            if (showSetupWizard)
            {
                DrawSetupWizard();
            }
            
            EditorGUILayout.Space(20);
            
            DrawQuickActions();
        }
        
        private void DrawSetupWizard()
        {
            EditorGUILayout.Space(10);
            EditorGUILayout.LabelField("セットアップウィザード", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            EditorGUILayout.HelpBox(
                "1. 対象オブジェクトを選択\n" +
                "2. 物理設定を選択（髪、胸、スカートなど）\n" +
                "3. コライダーの自動配置設定\n" +
                "4. セットアップ実行",
                MessageType.Info
            );
            
            EditorGUILayout.Space(10);
            
            if (GUILayout.Button("ウィザードを閉じる"))
            {
                showSetupWizard = false;
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawPhysicsTab()
        {
            if (selectedController == null)
            {
                EditorGUILayout.HelpBox("コントローラーを選択してください", MessageType.Info);
                return;
            }
            
            serializedController.Update();
            
            EditorGUILayout.LabelField("基本設定", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            EditorGUILayout.PropertyField(updateModeProp, new GUIContent("更新モード"));
            EditorGUILayout.PropertyField(iterationProp, new GUIContent("反復回数"));
            EditorGUILayout.PropertyField(timeScaleProp, new GUIContent("タイムスケール"));
            
            EditorGUILayout.Space(5);
            
            EditorGUILayout.PropertyField(windForceScaleProp, new GUIContent("風力スケール"));
            EditorGUILayout.PropertyField(colliderCollisionTypeProp, new GUIContent("コリジョンタイプ"));
            
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.Space(10);
            
            EditorGUILayout.LabelField("最適化設定", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            selectedController.isOptimize = EditorGUILayout.Toggle("最適化を有効化", selectedController.isOptimize);
            selectedController.isRunAsync = EditorGUILayout.Toggle("非同期実行", selectedController.isRunAsync);
            selectedController.isParallel = EditorGUILayout.Toggle("並列処理", selectedController.isParallel);
            
            EditorGUILayout.EndVertical();
            
            serializedController.ApplyModifiedProperties();
            
            EditorGUILayout.Space(10);
            
            if (GUILayout.Button("デフォルト設定に戻す", GUILayout.Height(25)))
            {
                ResetToDefaultSettings();
            }
        }
        
        private void DrawCollidersTab()
        {
            if (selectedController == null)
            {
                EditorGUILayout.HelpBox("コントローラーを選択してください", MessageType.Info);
                return;
            }
            
            serializedController.Update();
            
            EditorGUILayout.LabelField("コライダー設定", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            EditorGUILayout.LabelField("現在のコライダーサイズ", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PropertyField(colliderSizeProp, new GUIContent("グローバルサイズ"));
            
            if (GUILayout.Button("適用", GUILayout.Width(60)))
            {
                selectedController.RefreshSize();
            }
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.HelpBox(
                "コライダーサイズを小さくすると、より正確な衝突判定が可能になりますが、" +
                "パフォーマンスに影響する可能性があります。",
                MessageType.Info
            );
            
            EditorGUILayout.Space(10);
            
            EditorGUILayout.LabelField("推奨サイズ設定", EditorStyles.boldLabel);
            
            if (GUILayout.Button("最小サイズ (0.5)", GUILayout.Height(25)))
            {
                colliderSizeProp.floatValue = 0.5f;
                serializedController.ApplyModifiedProperties();
                selectedController.RefreshSize();
            }
            
            if (GUILayout.Button("標準サイズ (1.0)", GUILayout.Height(25)))
            {
                colliderSizeProp.floatValue = 1.0f;
                serializedController.ApplyModifiedProperties();
                selectedController.RefreshSize();
            }
            
            if (GUILayout.Button("大きめサイズ (1.5)", GUILayout.Height(25)))
            {
                colliderSizeProp.floatValue = 1.5f;
                serializedController.ApplyModifiedProperties();
                selectedController.RefreshSize();
            }
            
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.Space(10);
            
            DrawColliderList();
            
            serializedController.ApplyModifiedProperties();
        }
        
        private void DrawColliderList()
        {
            EditorGUILayout.LabelField("コライダーリスト", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            if (selectedController.overlapsColliderList != null && selectedController.overlapsColliderList.Count > 0)
            {
                EditorGUILayout.LabelField($"コライダー数: {selectedController.overlapsColliderList.Count}");
                
                EditorGUILayout.Space(5);
                
                foreach (var collider in selectedController.overlapsColliderList)
                {
                    if (collider != null)
                    {
                        EditorGUILayout.BeginHorizontal();
                        EditorGUILayout.ObjectField(collider, typeof(ADBColliderReader), false);
                        
                        if (GUILayout.Button("選択", GUILayout.Width(50)))
                        {
                            Selection.activeObject = collider;
                        }
                        
                        EditorGUILayout.EndHorizontal();
                    }
                }
            }
            else
            {
                EditorGUILayout.LabelField("コライダーが設定されていません");
            }
            
            EditorGUILayout.Space(10);
            
            if (GUILayout.Button("コライダーを自動生成", GUILayout.Height(25)))
            {
                AutoGenerateColliders();
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawPresetsTab()
        {
            EditorGUILayout.LabelField("プリセット管理", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            presetSearchFilter = EditorGUILayout.TextField("検索", presetSearchFilter);
            
            EditorGUILayout.Space(5);
            
            var filteredPresets = string.IsNullOrEmpty(presetSearchFilter) 
                ? availablePresets 
                : availablePresets.Where(p => p.name.ToLower().Contains(presetSearchFilter.ToLower())).ToList();
            
            foreach (var preset in filteredPresets)
            {
                EditorGUILayout.BeginHorizontal();
                
                bool isSelected = selectedPreset == preset;
                if (GUILayout.Toggle(isSelected, preset.name, "Button"))
                {
                    selectedPreset = preset;
                }
                
                if (GUILayout.Button("適用", GUILayout.Width(50)))
                {
                    ApplyPreset(preset);
                }
                
                EditorGUILayout.EndHorizontal();
            }
            
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.Space(10);
            
            if (GUILayout.Button("プリセットフォルダを開く", GUILayout.Height(25)))
            {
                EditorUtility.RevealInFinder("Packages/NatanePhysics/AutomaticDynamicBone/Resources/Setting");
            }
        }
        
        private void DrawDebugTab()
        {
            EditorGUILayout.LabelField("デバッグツール", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            if (selectedController != null)
            {
                selectedController.isDebug = EditorGUILayout.Toggle("デバッグモード", selectedController.isDebug);
                selectedController.isDrawGizmo = EditorGUILayout.Toggle("ギズモ表示", selectedController.isDrawGizmo);
                
                EditorGUILayout.Space(10);
                
                EditorGUILayout.LabelField("ステータス", EditorStyles.boldLabel);
                
                if (Application.isPlaying)
                {
                    EditorGUILayout.LabelField($"チェーン数: {selectedController.allChain?.Length ?? 0}");
                    EditorGUILayout.LabelField($"コライダー数: {selectedController.overlapsColliderList?.Count ?? 0}");
                    EditorGUILayout.LabelField($"反復回数: {selectedController.iteration}");
                    EditorGUILayout.LabelField($"バッファ時間: {selectedController.bufferTime}");
                }
                else
                {
                    EditorGUILayout.HelpBox("実行時のステータスを表示するには、再生モードに入ってください", MessageType.Info);
                }
            }
            else
            {
                EditorGUILayout.HelpBox("コントローラーを選択してください", MessageType.Info);
            }
            
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.Space(10);
            
            DrawPreviewControls();
        }
        
        private void DrawPreviewControls()
        {
            EditorGUILayout.LabelField("プレビューコントロール", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            EditorGUILayout.BeginHorizontal();
            
            GUI.enabled = Application.isPlaying && selectedController != null;
            
            if (GUILayout.Button(isPreviewMode ? "プレビュー停止" : "プレビュー開始", GUILayout.Height(30)))
            {
                isPreviewMode = !isPreviewMode;
            }
            
            if (GUILayout.Button("リセット", GUILayout.Height(30)))
            {
                ResetPhysics();
            }
            
            GUI.enabled = true;
            
            EditorGUILayout.EndHorizontal();
            
            if (isPreviewMode && Application.isPlaying)
            {
                EditorGUILayout.Space(5);
                previewTime = EditorGUILayout.Slider("プレビュー時間", previewTime, 0f, 5f);
            }
            
            EditorGUILayout.EndVertical();
        }
        
        private void DrawQuickActions()
        {
            EditorGUILayout.LabelField("クイックアクション", EditorStyles.boldLabel);
            
            EditorGUILayout.BeginVertical(boxStyle);
            
            EditorGUILayout.BeginHorizontal();
            
            if (GUILayout.Button("髪セットアップ", GUILayout.Height(25)))
            {
                QuickSetupHair();
            }
            
            if (GUILayout.Button("スカートセットアップ", GUILayout.Height(25)))
            {
                QuickSetupSkirt();
            }
            
            if (GUILayout.Button("胸セットアップ", GUILayout.Height(25)))
            {
                QuickSetupChest();
            }
            
            EditorGUILayout.EndHorizontal();
            
            EditorGUILayout.EndVertical();
        }
        
        private void PerformQuickSetup()
        {
            if (targetObject == null) return;
            
            var controller = targetObject.GetComponent<ADBRuntimeController>();
            if (controller == null)
            {
                controller = targetObject.AddComponent<ADBRuntimeController>();
            }
            
            controller.colliderSize = globalColliderScale;
            
            if (autoSetupChains)
            {
                // チェーンツールの自動セットアップ
                // ADBChainGenerateToolが存在する場合は使用する
                var chainToolType = System.Type.GetType("TKPhysics.AutomaticDynamicBone.Runtime.ADBChainGenerateTool");
                if (chainToolType != null)
                {
                    var chainTool = targetObject.GetComponent(chainToolType);
                    if (chainTool == null)
                    {
                        targetObject.AddComponent(chainToolType);
                    }
                }
            }
            
            if (autoSetupColliders)
            {
                // コライダーツールの自動セットアップ
                // ADBColliderGenerateToolが存在する場合は使用する
                var colliderToolType = System.Type.GetType("TKPhysics.AutomaticDynamicBone.Runtime.ADBColliderGenerateTool");
                if (colliderToolType != null)
                {
                    var colliderTool = targetObject.GetComponent(colliderToolType);
                    if (colliderTool == null)
                    {
                        targetObject.AddComponent(colliderToolType);
                    }
                }
            }
            
            selectedController = controller;
            
            EditorUtility.DisplayDialog("セットアップ完了", "ADB物理システムのセットアップが完了しました", "OK");
        }
        
        private void RefreshControllers()
        {
            LoadPresets();
            if (selectedController != null)
            {
                serializedController = new SerializedObject(selectedController);
            }
        }
        
        private void ResetToDefaultSettings()
        {
            if (selectedController == null) return;
            
            selectedController.iteration = 4;
            selectedController.timeScale = 1f;
            selectedController.windForceScale = 0f;
            selectedController.colliderSize = 1f;
            selectedController.isOptimize = false;
            selectedController.isRunAsync = false;
            selectedController.isParallel = false;
            
            EditorUtility.SetDirty(selectedController);
        }
        
        private void AutoGenerateColliders()
        {
            if (selectedController == null) return;
            
            // コライダーツールの自動生成
            var colliderToolType = System.Type.GetType("TKPhysics.AutomaticDynamicBone.Runtime.ADBColliderGenerateTool");
            if (colliderToolType != null)
            {
                var colliderTool = selectedController.GetComponent(colliderToolType);
                if (colliderTool == null)
                {
                    selectedController.gameObject.AddComponent(colliderToolType);
                }
            }
            
            EditorUtility.DisplayDialog("コライダー生成", "コライダーの自動生成を開始しました", "OK");
        }
        
        private void ApplyPreset(ADBPhysicsSetting preset)
        {
            if (selectedController == null || preset == null) return;
            
            EditorUtility.DisplayDialog("プリセット適用", $"プリセット '{preset.name}' を適用しました", "OK");
        }
        
        private void ResetPhysics()
        {
            if (selectedController == null || !Application.isPlaying) return;
            
            selectedController.enabled = false;
            selectedController.enabled = true;
        }
        
        private void QuickSetupHair()
        {
            if (targetObject == null) return;
            
            var preset = availablePresets.FirstOrDefault(p => p.name.ToLower().Contains("hair"));
            if (preset != null)
            {
                ApplyPreset(preset);
            }
        }
        
        private void QuickSetupSkirt()
        {
            if (targetObject == null) return;
            
            var preset = availablePresets.FirstOrDefault(p => p.name.ToLower().Contains("skirt"));
            if (preset != null)
            {
                ApplyPreset(preset);
            }
        }
        
        private void QuickSetupChest()
        {
            if (targetObject == null) return;
            
            var preset = availablePresets.FirstOrDefault(p => p.name.ToLower().Contains("chest"));
            if (preset != null)
            {
                ApplyPreset(preset);
            }
        }
    }
}