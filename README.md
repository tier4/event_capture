# event capture

## 機能
Rvizの3D画面にマウスオーバーした時のマウスの動作を出力

## Publish Topic
/rviz/event_capture/mouse (event_capture/MouseEventCaptureStamped)

## 使用方法
1. vizのツールバー（Interactのあるバー）で
＋ボタン -> Event Capture を選択 -> OK

1. `rostopic echo /rviz/event_capture/mouse` 
を実行し、Rvizの3D画面でマウスを動かしてトピックの出力を見てみる

## Special Thanks
@isamu-takagi
