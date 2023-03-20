# joy2can
joyトピックをcanに変換するノード

## Hardware Interfaces

すべてのメッセージは標準フレームフォーマットで送受信される。

### PCからマイコンに送信するメッセージ
* ID: 20
```c++
struct JoyMsg {
  int8_t left_stick_x;
  int8_t left_stick_y;
  int8_t right_stick_x;
  int8_t right_stick_y;
  int8_t l2;
  int8_t r2;
  uint8_t buttons_1;
  uint8_t buttons_2;
} __attribute__((packed));
```

##### buttons_1
* ビット0: ×
* ビット1: ○
* ビット2: □
* ビット3: △
* ビット4: Share
* ビット5: PS
* ビット6: Option
* ビット7: L3

##### buttons_2
* ビット0: R3
* ビット1: L1
* ビット2: R1
* ビット3: DPad-UP
* ビット4: DPad-Down
* ビット5: DPad-Left
* ビット6: DPad-Right
* ビット7: Touchpad

##### アナログスティックについて
* x軸方向が左、y軸方向が上
* スティックが中央にあるとき、x、yはともに0
* 例:
  * ステイックが中央にあるとき → (x,y)=(0,0)
  * ステイックを右に倒したとき → (x,y)=(-127,0)
  * ステイックを下に倒したとき → (x,y)=(0,-127)
  * ステイックを左上に倒したとき → (x,y)=(127,127)

##### L2、R2ボタンについて
* 初期位置が0、完全に押されると255となる。
