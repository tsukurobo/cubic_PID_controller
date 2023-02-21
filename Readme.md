# Readme

*Due to breaking changes in Cubic, this library is outdated.*

このライブラリは、Cubic用のPID制御ライブラリです。

バージョン1.8のCubicライブラリに依存しています。

[WEBドキュメント](https://tsukurobo.github.io/cubic_PID_controller/html/)

## Update history

- ver 0.20
  - Cubicライブラリの依存バージョンを1.8に更新 #3
    - エンコーダーの値の型をint32_tに変更
  - setTarget()の引数の型をdoubleに修正 #9
- ver 0.16
  - Anti-windupを導入。
- ver 0.15
  - compute()内でpreDiffが更新されてなかったバグを修正。
- ver 0.142
  - 一部の変数名をわかりやすいように変更。
  - ドキュメントの説明を一部修正。
- ver 0.141
  - プログラム内のバージョン番号を修正。
- ver 0.14
  - compute()内でifPrint==falseの時にも一部のログが出力されていたバグを修正
- ver 0.13
  - エンコーダの分解能を設定できる機能を追加
- ver 0.12
  - 方向を変更できるように修正
  - サンプルプログラムでput, sendしていなかったのを修正