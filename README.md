# CRANE-X7_cigarette_dispenser_robot

## 概要
CRANE-X7とmediaPipeを連携させてシガレットをピッキングする開発中のROS2パッケージです.

- **画像認識**    : mediaPipeを用いて、指の本数を検知してトリガーとする.
- **アームの制御**: シガレットのピッキング動作を行う.

## デモ



## 使用方法

### リポジトリのクローン

```bash
git clone https://github.com/2025-RobotDesign3-YFK/cigarette_dispenser_robot.gitt
```

### Gazeboで実行する場合

- **terminal 1:** `bash ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py`
- **terminal 2:** `ros2 launch cigarette_dispenser_robot detection_and_motion.launch.py use_sim_time:='true'`

### 実機で実行する場合

- **terminal 1:** `ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0`
- **terminal 2:** `ros2 launch cigarette_dispenser_robot detection_and_motion.launch.py`


## ライセンス
- © 2025 Hiroto Fujitake, Syougo Yamasita, Reoto Koya
- このパッケージは、CIT-Autonomous-Robot-Labの公開する[パッケージ](https://github.com/CIT-Autonomous-Robot-Lab/crane_x7_simple_examples)の[src](https://github.com/CIT-Autonomous-Robot-Lab/crane_x7_simple_examples/blob/main/src)のpick_and_move.cppの改変によって作成されました.
- このパッケージはApache License, Version 2.0に基づき公開されています.  
- ライセンスの全文は[LICENSE](./LICENSE)から確認できます.  


