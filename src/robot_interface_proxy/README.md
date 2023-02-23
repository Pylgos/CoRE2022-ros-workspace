## Protocol Specification

数値はすべてリトルエンディアンである。
すべてのメッセージは標準フレームフォーマットで送信される。


### PCからマイコンに送信するメッセージ
* ID: 20 target_velocity
```c++
struct TargetVelocityMsg {
  int16_t vx; // 前後方向の速度[m/s] * 1000 前が+　後ろが-
  int16_t vy; //　左右方向の速度[m/s] * 1000 左が+ 右が-
  int16_t ang_vel; // 回転速度[rad/s] * 1000 左旋回が+ 右旋回が-
}
// 例: 前に1[m/s]、右に1[m/s]、右旋回1[rad/s]とき、vx==1000, vy==-1000, ang_vel==-1000
```

* ID: 21 camera_angle
```c++
struct CameraAngleMsg {
  int16_t pitch; // 上下方向の角度[rad] * 1000 上が-　下が+
  int16_t yaw; // 左右方向の角度[rad] * 1000 左が+ 右が-
}
// 例: 上30[°], 左70[°]のとき、pitch == -30 / 180 * M_PI * 1000, yaw == 70 / 180 * M_PI * 1000
```

### マイコンからPCに送信するメッセージ
* ID: 22 launcher_info
```c++
struct LauncherInfoMsg {
  uint8_t ammo; // 残弾数
}
```
