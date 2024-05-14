# UnityPhysicsSetter
Simple Physics

個人で使いたい時用に用意したので、説明が少ないです。

Unity2022.3系での動作しか確認していません。
物理計算の最適化ではなく、どこにでもあるような物理計算をBurstコンパイルとJobSystemを使った実装に変換しました。
主に使うものは３つ・すべて親にコンポーネントをつけて使用する
・ADBRuntimeController
　コントローラー
・ADBColliderGenerateTool
　コライダーを自動生成
・ADBChainGenerateTool
　つながりを自動生成。

https://github.com/natane010/UnityPhysicsSetter.git?path=/Packages/TKPhysics
