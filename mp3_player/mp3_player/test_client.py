#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mp3_player_msg.msg import SoundPlay

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client')
        self.publisher = self.create_publisher(SoundPlay, 'audio_command', 10)
        
    def send_play_command(self, file_path):
        """MP3ファイルを再生するコマンドを送信"""
        msg = SoundPlay()
        msg.mode = SoundPlay.PLAY
        msg.file_path = file_path
        msg.volume = 1.0
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent PLAY command for: {file_path}')
        
    def send_stop_command(self):
        """停止コマンドを送信"""
        msg = SoundPlay()
        msg.mode = SoundPlay.STOP
        msg.file_path = ""
        msg.volume = 1.0
        self.publisher.publish(msg)
        self.get_logger().info('Sent STOP command')
        
    def send_pause_command(self):
        """一時停止コマンドを送信"""
        msg = SoundPlay()
        msg.mode = SoundPlay.PAUSE
        msg.file_path = ""
        msg.volume = 1.0
        self.publisher.publish(msg)
        self.get_logger().info('Sent PAUSE command')
        
    def send_unpause_command(self):
        """再生再開コマンドを送信"""
        msg = SoundPlay()
        msg.mode = SoundPlay.UNPAUSE
        msg.file_path = ""
        msg.volume = 1.0
        self.publisher.publish(msg)
        self.get_logger().info('Sent UNPAUSE command')
        
    def send_volume_command(self, volume):
        """音量設定コマンドを送信"""
        msg = SoundPlay()
        msg.mode = SoundPlay.SET_VOLUME
        msg.file_path = ""
        msg.volume = volume
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent SET_VOLUME command: {volume}')

def main():
    rclpy.init()
    client = TestClient()
    
    # 使用例
    import time
    
    # 1秒待ってからコマンドを送信（ノードが準備できるまで）
    time.sleep(1)
    
    # 音量を0.5に設定
    client.send_volume_command(0.5)
    time.sleep(0.5)
    
    # MP3ファイルを再生（実際のファイルパスに変更してください）
    # client.send_play_command('/path/to/your/music.mp3')
    
    # テスト用のダミーコマンド
    client.send_play_command('/tmp/test.mp3')
    time.sleep(2)
    
    client.send_pause_command()
    time.sleep(2)
    
    client.send_unpause_command()
    time.sleep(2)
    
    client.send_stop_command()
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
