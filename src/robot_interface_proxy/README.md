# robot_interface_proxy
ROSとマイコン間の通信を中継するためのパッケージ

## ROS Interfaces
#### Publish
- `std_msgs/Int64 ammo`

#### Subscribe
- `geometry_msgs/Twist target_vel`
- `geometry_msgs/Vector3 camera_angle`
`x`, `y`, `z` がそれぞれ `roll`, `pitch`, `yaw` に対応する。


#### Parameters
##### CANProxy node
- `double read_rate`
- `double write_rate`
- `double target_vel_expire_duration`
- `string can_interface`
- `int target_vel_can_id`
- `int camera_angle_can_id`
- `int launcher_info_can_id`


## Hardware Interfaces

数値はすべてリトルエンディアンである。
すべてのメッセージは標準フレームフォーマットで送受信される。
`target_velocity`はマイコンが直接コントローラと通信する（ROSを経由して操作しない）場合、無視する。

### PCからマイコンに送信するメッセージ
* ID: 20 target_velocity  
ロボットの移動する目標速度を表すメッセージ  
```c++
struct TargetVelocityMsg {
  int16_t vx; // 前後方向の速度[m/s] * 1000 前が+　後ろが-
  int16_t vy; //　左右方向の速度[m/s] * 1000 左が+ 右が-
  int16_t ang_vel; // 回転速度[rad/s] * 1000 左旋回が+ 右旋回が-
} __attribute__((packed));
// 例: 前に1[m/s]、右に1[m/s]、右旋回1[rad/s]とき、vx==1000, vy==-1000, ang_vel==-1000
```

* ID: 21 camera_angle  
カメラを向ける角度を表すメッセージ
```c++
struct CameraAngleMsg {
  int16_t pitch; // 上下方向の角度[rad] * 1000 上が-　下が+
  int16_t yaw; // 左右方向の角度[rad] * 1000 左が+ 右が-
} __attribute__((packed));
// 例: 上30[°], 左70[°]のとき、pitch == -30 / 180 * M_PI * 1000, yaw == 70 / 180 * M_PI * 1000
```

* ID: 23 fire_command
```c++
struct FireCommandMsg {
  bool enable; // 発射を行うかどうか trueなら発射する
} __attribute__((packed));
```

* ID: 24 expand_camera
```c++
struct ExpandCameraMsg {
  // 空 (長さ0のメッセージ)
  // このメッセージを受信したとき、カメラを展開する
};
```

### マイコンからPCに送信するメッセージ
* ID: 22 launcher_info  
発射・装填機構の情報を表すメッセージ
```c++
struct LauncherInfoMsg {
  uint8_t ammo; // 残弾数
} __attribute__((packed));
```