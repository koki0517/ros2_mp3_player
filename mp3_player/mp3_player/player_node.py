
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mp3_player_msg.msg import SoundPlay
import pygame

class Mp3PlayerNode(Node):
    def __init__(self):
        super().__init__('mp3_player_node')
        self.subscription = self.create_subscription(
            SoundPlay,
            'audio_command',
            self.command_callback,
            10
        )
        pygame.mixer.init()
        self.current_file = None
        self.is_paused = False
        self.get_logger().info('mp3_player_node started.')

    def command_callback(self, msg):
        cmd = msg.mode
        if cmd == SoundPlay.PLAY:
            self.play_file(msg.file_path)
        elif cmd == SoundPlay.STOP:
            self.stop()
        elif cmd == SoundPlay.PAUSE:
            self.pause()
        elif cmd == SoundPlay.UNPAUSE:
            self.unpause()
        elif cmd == SoundPlay.SET_VOLUME:
            self.set_volume(msg.volume)
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')

    def play_file(self, file_path):
        try:
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            self.current_file = file_path
            self.is_paused = False
            self.get_logger().info(f'Playing: {file_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to play {file_path}: {e}')

    def stop(self):
        pygame.mixer.music.stop()
        self.current_file = None
        self.is_paused = False
        self.get_logger().info('Playback stopped.')

    def pause(self):
        if pygame.mixer.music.get_busy() and not self.is_paused:
            pygame.mixer.music.pause()
            self.is_paused = True
            self.get_logger().info('Playback paused.')

    def unpause(self):
        if self.is_paused:
            pygame.mixer.music.unpause()
            self.is_paused = False
            self.get_logger().info('Playback unpaused.')

    def set_volume(self, volume):
        v = max(0.0, min(1.0, volume))
        pygame.mixer.music.set_volume(v)
        self.get_logger().info(f'Volume set to {v}')

def main(args=None):
    rclpy.init(args=args)
    node = Mp3PlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.mixer.quit()

if __name__ == '__main__':
    main()
