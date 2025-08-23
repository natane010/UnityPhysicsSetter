# NatanePhysics - Advanced Dynamic Bone System

個人で使いたい時用に用意したので、説明が少ないです。
説明→https://qiita.com/natane/private/c562ec0ee5abc5f2b540

使い方などは[こちら](https://github.com/natane010/UnityPhysicsSetter/blob/develop/2022_3/Packages/NatanePhysics/README.md)

下をパッケージマネージャーのGitURLに入力するとパッケージとして入れられます。
https://github.com/natane010/UnityPhysicsSetter.git?path=/Packages/NatanePhysics


# NatanePhysics - Advanced Dynamic Bone System

[![Unity](https://img.shields.io/badge/Unity-2022.3%2B-blue.svg)](https://unity3d.com)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-1.2.3-orange.svg)](package.json)

## 📋 概要

NatanePhysics (旧称: UnityNataneDynamicBone) は、Unity向けの高性能な物理ベースの揺れものシステムです。髪、服、アクセサリーなどのダイナミックな動きを、Unity's Job SystemとBurst Compilerを活用して効率的にシミュレートします。

### ✨ 主な特徴

- **高性能**: Burst CompilerとJob Systemによる最適化
- **MagicaCloth2インスパイア**: 業界標準の技術を参考にした高度な物理シミュレーション
- **豊富な制約システム**: 6種類の制約タイプによる自然な動き
- **LODシステム**: カメラ距離に応じた自動品質調整
- **高度な風システム**: 5つの風モードと自然な乱流シミュレーション
- **セルフコリジョン**: 自己衝突の検出と解決
- **使いやすいエディタ**: 専用のEditorWindowによる簡単なセットアップ

## 🚀 クイックスタート

### インストール

1. **Package Manager経由でのインストール**
   ```
   Window > Package Manager > + > Add package from git URL...
   https://github.com/natane010/UnityPhysicsSetter.git?path=/Packages/NatanePhysics
   ```

2. **手動インストール**
   - `/Packages/NatanePhysics`フォルダを自分のプロジェクトの`Packages`フォルダにコピー

### 基本的な使い方

1. **コンポーネントの追加**
   - 揺らしたいオブジェクトの親に`ADBRuntimeController`を追加
   - 必要に応じて`ADBChainGenerateTool`と`ADBColliderGenerateTool`を追加

2. **EditorWindowでセットアップ**
   - `Window > ADB Physics Manager`を開く
   - 対象オブジェクトを選択
   - 「セットアップ実行」をクリック

3. **プリセットの適用**
   - 髪、スカート、胸などの部位別プリセットを選択
   - 「適用」ボタンで即座に反映

## 🏗️ アーキテクチャ

### コア構造

```
NatanePhysics/
├── Runtime/                        # ランタイムコード
│   ├── Core/                      # コア物理エンジン
│   │   ├── ADBPhysicsKernel.cs   # メイン物理計算
│   │   ├── ADBRunTimeJobsTable.cs # Jobシステム実装
│   │   └── ADBRuntimeController.cs # メインコントローラー
│   ├── Constraints/               # 制約システム
│   │   ├── ADBRuntimeConstraint.cs
│   │   └── ADBAdvancedConstraints.cs
│   ├── Systems/                   # 追加システム
│   │   ├── ADBStabilitySystem.cs  # 安定性システム
│   │   ├── ADBLODSystem.cs       # LODシステム
│   │   ├── ADBAdvancedWind.cs    # 高度な風システム
│   │   └── ADBSolverOptimization.cs # ソルバー最適化
│   └── Data/                      # データ構造
│       ├── ADBPhysicsSetting.cs  # 物理設定
│       └── PointStruct.cs        # ポイントデータ
├── Editor/                        # エディタ拡張
│   ├── ADBPhysicsManagerWindow.cs # メインエディタWindow
│   └── ADBColliderSizeOptimizer.cs # コライダー最適化ツール
└── Resources/                     # リソース
    └── Setting/                   # プリセット設定
        └── MMD/                   # MMD用プリセット
            ├── Low/              # 低品質設定
            ├── Medium/           # 中品質設定
            └── High/             # 高品質設定
```

## 🔧 主要コンポーネント

### ADBRuntimeController
物理シミュレーションのメインコントローラー。すべての物理計算を管理します。

**主要パラメータ:**
- `iteration`: 反復回数 (1-10)
- `timeScale`: タイムスケール
- `windForceScale`: 風力の強度
- `colliderSize`: グローバルコライダーサイズ

### ADBChainGenerateTool
ボーンチェーンの自動生成ツール。階層構造を解析して物理チェーンを構築します。

### ADBColliderGenerateTool
コライダーの自動生成ツール。メッシュやボーン構造からコライダーを自動配置します。

## 🎯 制約システム

### 実装済み制約タイプ

1. **Structural Constraints (構造制約)**
   - Vertical: 垂直方向の接続
   - Horizontal: 水平方向の接続

2. **Shear Constraints (せん断制約)**
   - 対角線方向の変形を制御

3. **Bending Constraints (曲げ制約)**
   - Vertical: 垂直方向の曲げ
   - Horizontal: 水平方向の曲げ

4. **Circumference Constraints (円周制約)**
   - 円形構造の維持

5. **Advanced Constraints (高度な制約)**
   - Tether: 最大距離制限
   - Angle: 角度制限と復元
   - Triangle Bending: 三角形曲げ
   - Motion: 移動制限とバックストップ

## 🌬️ 高度な風システム

### 風モード
- **GlobalDirection**: グローバル方向風
- **SphereDirection**: 球体方向風
- **BoxDirection**: 箱型方向風
- **SphereRadial**: 球体放射風
- **Vortex**: 渦巻き風

### 波形タイプ
- Sine波
- Noise波
- Pulse波
- Random波

## 📊 パフォーマンス最適化

### LODシステム
カメラ距離とパフォーマンスに基づく動的品質調整：
- **Distance Mode**: 距離ベースのLOD
- **Performance Mode**: FPSベースの自動調整
- **Hybrid Mode**: 両方の組み合わせ

### 最適化技術
- **Burst Compiler**: 高速な物理計算
- **Job System**: 並列処理による効率化
- **空間分割**: 効率的な衝突検出
- **階層的ソルバー**: レベル別の段階的計算
- **適応的品質調整**: リアルタイムの品質最適化

## 🎨 プリセット

### 部位別プリセット
- **Hair (髪)**: 柔らかく自然な髪の動き
- **Skirt (スカート)**: 布の物理シミュレーション
- **Chest (胸)**: 適度な弾性と減衰
- **Accessory (アクセサリー)**: 軽量で反応的な動き

### 品質レベル
- **Low**: パフォーマンス重視
- **Medium**: バランス型
- **High**: 品質重視

## 🔬 技術仕様

### 要件
- Unity 2022.3 LTS以降
- Universal Render Pipeline (URP) 対応
- Burst Compiler Package
- Mathematics Package
- Collections Package

### パフォーマンス指標
- **処理能力**: 1000+ パーティクル @ 60FPS (標準的なPC)
- **メモリ使用**: ~10MB per 1000 particles
- **初期化時間**: < 100ms

## 🆚 MagicaCloth2との比較

| 機能 | NatanePhysics | MagicaCloth2 |
|------|--------------|--------------|
| **価格** | 無料/オープンソース | 有料 |
| **制約タイプ** | 6種類+ | 8種類+ |
| **LODシステム** | ✅ | ✅ |
| **セルフコリジョン** | ✅ | ✅ |
| **風システム** | 高度（5モード） | 基本 |
| **安定性システム** | ✅ | ✅ |
| **カスタマイズ性** | 高（ソースコード公開） | 中 |

## 📝 ライセンス

MIT License - 商用利用可能

## 🤝 コントリビューション

プルリクエストを歓迎します！
バグ報告や機能リクエストは[Issues](https://github.com/natane010/UnityPhysicsSetter/issues)まで。

## 📚 ドキュメント

詳細なドキュメントは[Wiki](https://github.com/natane010/UnityPhysicsSetter/wiki)を参照してください。

## 🙏 謝辞

- Unity Technologies - Job SystemとBurst Compiler
- MagicaSoft - MagicaCloth2からのインスピレーション
- コミュニティの貢献者の皆様

---

**Developed with ❤️ by natane010**