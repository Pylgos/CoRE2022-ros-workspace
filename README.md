# CoRE2023 Team B ROS Workspace

これはCoRE2023のBチームで使用されるROSワークスペースです。

## Build
開発環境はNixで管理されています。下記の手順に従って開発環境をセットアップできます。
* [Nix](https://nixos.org/download.html)をインストールする
* [Flake](https://nixos.wiki/wiki/Flakes)を有効化する
* `git clone https://github.com/omuct-robotclub/CoRE2023-team-b-ros-workspace`
* `cd CoRE2023-team-b-ros-workspace`
* `nix develop`で開発環境に入る
* `colcon build`でビルドを行う


## Note
メインマイコンとの通信プロトコルは[robot_interface_proxyのREADME](./src/robot_interface_proxy/README.md)と[joy2canのREADME](./src/joy2can/README.md)にあります
