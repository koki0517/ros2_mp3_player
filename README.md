# ros2_mp3_player

## 概要
ROS2上でMP3ファイルの再生・停止・一時停止・音量調整などを行うノード

- mp3_player: MP3再生ノード
- mp3_player_msg: トピック

## 環境
- ROS 2 Humble
- Python3
- pygame

## ノードの起動
```bash
ros2 run mp3_player mp3_player_node
```

## トピック
### 1. コマンドラインから
```bash
ros2 topic pub --once /audio_command mp3_player_msg/msg/SoundPlay "{mode: 0, file_path: '/path/to/music.mp3', volume: 1.0}"
ros2 topic pub --once /audio_command mp3_player_msg/msg/SoundPlay "{mode: 1, file_path: '', volume: 1.0}"
ros2 topic pub --once /audio_command mp3_player_msg/msg/SoundPlay "{mode: 2, file_path: '', volume: 1.0}"
ros2 topic pub --once /audio_command mp3_player_msg/msg/SoundPlay "{mode: 3, file_path: '', volume: 1.0}"
ros2 topic pub --once /audio_command mp3_player_msg/msg/SoundPlay "{mode: 4, file_path: '', volume: 0.5}"
```

### 2. Pythonクライアントから
```bash
ros2 run mp3_player audio_client play /path/to/music.mp3
ros2 run mp3_player audio_client stop
ros2 run mp3_player audio_client pause
ros2 run mp3_player audio_client unpause
ros2 run mp3_player audio_client volume 0.5
```

## 使用例
### MP3再生
```bash
ros2 topic pub --once /audio_command mp3_player_msg/msg/SoundPlay "{mode: 0, file_path: '/home/user/music.mp3', volume: 1.0}"
```

### 音量調整
```bash
ros2 topic pub --once /audio_command mp3_player_msg/msg/SoundPlay "{mode: 4, file_path: '', volume: 0.5}"
```