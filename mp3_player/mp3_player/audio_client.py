#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mp3_player_msg.msg import SoundPlay
import sys
import os

class AudioClient(Node):
    def __init__(self):
        super().__init__('audio_client')
        self.publisher = self.create_publisher(SoundPlay, 'audio_command', 10)
        # パブリッシャーの準備ができるまで少し待機
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.ready = False
        
    def timer_callback(self):
        if not self.ready:
            self.ready = True
            self.timer.cancel()
        
    def play(self, file_path, volume=1.0):
        """MP3ファイルを再生"""
        if not os.path.exists(file_path):
            self.get_logger().error(f'ファイルが見つかりません: {file_path}')
            return False
            
        msg = SoundPlay()
        msg.mode = SoundPlay.PLAY
        msg.file_path = file_path
        msg.volume = max(0.0, min(1.0, volume))  # 0.0-1.0の範囲に制限
        
        self.publisher.publish(msg)
        self.get_logger().info(f'再生開始: {file_path} (音量: {msg.volume})')
        return True
        
    def stop(self):
        """再生を停止"""
        msg = SoundPlay()
        msg.mode = SoundPlay.STOP
        msg.file_path = ""
        msg.volume = 1.0
        
        self.publisher.publish(msg)
        self.get_logger().info('再生を停止しました')
        
    def pause(self):
        """再生を一時停止"""
        msg = SoundPlay()
        msg.mode = SoundPlay.PAUSE
        msg.file_path = ""
        msg.volume = 1.0
        
        self.publisher.publish(msg)
        self.get_logger().info('再生を一時停止しました')
        
    def unpause(self):
        """再生を再開"""
        msg = SoundPlay()
        msg.mode = SoundPlay.UNPAUSE
        msg.file_path = ""
        msg.volume = 1.0
        
        self.publisher.publish(msg)
        self.get_logger().info('再生を再開しました')
        
    def set_volume(self, volume):
        """音量を設定"""
        msg = SoundPlay()
        msg.mode = SoundPlay.SET_VOLUME
        msg.file_path = ""
        msg.volume = max(0.0, min(1.0, volume))  # 0.0-1.0の範囲に制限
        
        self.publisher.publish(msg)
        self.get_logger().info(f'音量を{msg.volume}に設定しました')

def main():
    rclpy.init()
    client = AudioClient()
    
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python3 audio_client.py play <ファイルパス> [音量]")
        print("  python3 audio_client.py stop")
        print("  python3 audio_client.py pause")
        print("  python3 audio_client.py unpause")
        print("  python3 audio_client.py volume <0.0-1.0>")
        print()
        print("例:")
        print("  python3 audio_client.py play /home/user/music.mp3")
        print("  python3 audio_client.py play /home/user/music.mp3 0.5")
        print("  python3 audio_client.py volume 0.8")
        return
    
    command = sys.argv[1].lower()
    
    # パブリッシャーの準備を待つ
    import time
    time.sleep(0.2)
    
    if command == "play":
        if len(sys.argv) < 3:
            print("エラー: ファイルパスを指定してください")
            return
        file_path = sys.argv[2]
        volume = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
        client.play(file_path, volume)
        
    elif command == "stop":
        client.stop()
        
    elif command == "pause":
        client.pause()
        
    elif command == "unpause":
        client.unpause()
        
    elif command == "volume":
        if len(sys.argv) < 3:
            print("エラー: 音量値(0.0-1.0)を指定してください")
            return
        try:
            volume = float(sys.argv[2])
            client.set_volume(volume)
        except ValueError:
            print("エラー: 音量は数値で指定してください")
            
    else:
        print(f"エラー: 未知のコマンド '{command}'")
    
    # 少し待ってからシャットダウン
    time.sleep(0.1)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
