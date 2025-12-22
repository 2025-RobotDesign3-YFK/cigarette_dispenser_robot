# CRANE-X7_cigarette_dispenser_robot

## 概要
CRANE-X7とMediaPipeを連携させ,指で表現した数字に応じたシガレットをピッキングするROS2パッケージです.

- **画像認識**    : MediaPipeを用いて、指で表現された数を判定.
- **アームの制御**: 判定結果に基づいて、数に対応するシガレットのピッキング動作を行う.

## デモ

https://github.com/user-attachments/assets/0f1407a0-f818-47fb-8144-fadb2eecd3ec

## 動作環境

- **OS**: Ubuntu 22.04 LTS
- **ROS Version**: ROS 2 Humble
- **シュミレータ**: Gazebo
- **ライブラリ**: MediaPipe, OpenCV 
- **ハードウェア**: CRANE-X7

## セットアップ

### 1.ROS 2及びCRANE-X7セットアップ

**ROS 2インストール**
　　上田先生の[動画](https://youtu.be/mBhtD08f5KY)及び[インストールスクリプト](https://github.com/ryuichiueda/ros2_setup_scripts)を参照し, インストールを行ってください.
**CRANE-X7及び関連パッケージのインストール**
　　[RT社公式リポジトリ](https://github.com/rt-net/crane_x7_ros/tree/ros2)よりインストールできます.

### 2.リポジトリのクローン

```bash
cd ~/ros2_ws/src 
git clone https://github.com/2025-RobotDesign3-YFK/cigarette_dispenser_robot.git
```

### 3.MediaPipeのインストール

- **pipのインストール**
 ```bash
 sudo apt install python3-pip
 ```
- **ros2_wsディレクトリ内でMediaPipeをインストール**
 ```bash
 #ROS 2ディレクトリに移動
 cd ~/ros2_ws

 # pip自体のアップグレード（念のため）
 pip3 install --upgrade pip

 # OpenCVとMediaPipeのインストール
 pip3 install opencv-contrib-python mediapipe==0.10.8
 ```

## 使用方法

### Gazeboで実行する場合

- **terminal 1（ドライバ起動）:**
 ```bash
 ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
 ```
- **terminal 2（アプリ起動）:**
 ```bash
 ros2 launch cigarette_dispenser_robot detection_and_motion.launch.py use_sim_time:='true'
 ```

### 実機で実行する場合

- **USBの権限を設定**
 ```bash
 sudo chmod 666 /dev/ttyUSB0
 ```
- **terminal 1（ドライバ起動）:**
 ```bash
 ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
 ```
- **terminal 2（アプリ起動）:**
 ```bash
 ros2 launch cigarette_dispenser_robot detection_and_motion.launch.py
 ```


## ライセンス
- © 2025 Hiroto Fujitake, Shogo Yamashita, Reoto Koya
- このパッケージは、[CIT-Autonomous-Robot-Lab](https://github.com/CIT-Autonomous-Robot-Lab)の公開する[パッケージ](https://github.com/CIT-Autonomous-Robot-Lab/crane_x7_simple_examples)の[src](https://github.com/CIT-Autonomous-Robot-Lab/crane_x7_simple_examples/blob/main/src)のpick_and_move.cppと、[launch](https://github.com/CIT-Autonomous-Robot-Lab/crane_x7_simple_examples/blob/main/launch)のpick_and_move.launch.pyの改変によって作成されています.
- このパッケージの検知プログラムは、[matsuyamayusaku](https://github.com/matsuyamayusaku)の公開する[パッケージ](https://github.com/matsuyamayusaku/robdesstamp_2022.git)の[scripts](https://github.com/matsuyamayusaku/robdesstamp_2022/tree/main/scripts)のhand.pyの改変によって作成されました.
- このパッケージはApache License, Version 2.0に基づき公開されています.  
- ライセンスの全文は[LICENSE](./LICENSE)から確認できます.  


