# aps

## Hardware Interfaces
すべてのメッセージは標準フレームフォーマットで送受信される。

### PCからマイコンに送信するメッセージ
* ID: 26
```c++
struct ApsControl {
  int16_t motor_command; // モーターの指令値
  uint8_t servo_command; // サーボモータの指令値　サーボモータードライバで用いられるものと同じフォーマット
} __attribute__((packed));
```
