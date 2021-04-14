# event capture

## 機能
Rvizの3D画面で、右クリックしながらマウスオーバーした時のマウスの動作を出力

なお、左クリックは `Interact` を押している時と動作は変わらない

## Publish Topic
/rviz/event_capture/mouse (event_capture/MouseEventCaptureStamped)

## 使用方法
1. Rvizのツールバー（Interactのあるバー）で
＋ボタン -> Event Capture を選択 -> OK

1. Rvizのツールバーで Event Capture を選択 or `c` を入力

1. `rostopic echo /rviz/event_capture/mouse` 
を実行し、Rvizの3D画面で右クリックしながらマウスを動かしてトピックの出力を見てみる

## Special Thanks
@isamu-takagi
