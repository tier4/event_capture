# event capture

## 機能
Rvizの3D画面で、以下の2機能を実現
- 右クリックしながらマウスオーバーした時のマウスの動作を出力
- キーボード入力を出力（rvizキーボード・ショートカットに設定されているキーは出力不可能）

なお、左クリックは `Interact` を押している時と動作は変わらない

## Publish Topic
`/rviz/event_capture (event_capture/EventCaptureStamped)`

## 使用方法
1. Rvizのツールバー（Interactのあるバー）で
＋ボタン -> Event Capture を選択 -> OK

1. Rvizのツールバーで Event Capture を選択 or `c` を入力

1. `rostopic echo /rviz/event_capture`
を実行し、Rvizの3D画面で右クリックしながらマウスを動かしてトピックの出力を見てみる or キーボード入力を実施する

## Special Thanks

[@isamu-takagi](https://github.com/isamu-takagi)
