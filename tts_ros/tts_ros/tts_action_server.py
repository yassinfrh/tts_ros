import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from tts_ros_interfaces.action import TTS

from pyht import Client
from dotenv import load_dotenv
from pyht.client import TTSOptions
import os

class TTSActionServer(Node):

    def __init__(self):
        super().__init__('tts_action_server')
        self._action_server = ActionServer(
            self,
            TTS,
            'tts',
            self.execute_callback
        )
        
        self.client = Client(
            user_id=os.getenv('PLAY_HT_USER_ID'),
            api_key=os.getenv('PLAY_HT_API_KEY')
        )
        
        self.options = TTSOptions(voice="s3://voice-cloning-zero-shot/7b97b543-7877-41b6-86ee-aa1e0b6c110e/dicksaad/manifest.json")
        
        self.get_logger().info('TTS Action Server has been started')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        text = goal_handle.request.text
        
        file_path = os.path.expanduser('~/output.wav')
        # Create file if it doesn't exist
        open(file_path, 'w').close()
        
        with open(file_path, 'wb') as f:
            for chunk in self.client.tts(text, self.options, voice_engine='Play3.0-mini'):
                f.write(chunk)
                
        goal_handle.succeed()
        result = TTS.Result()
        result.result = True
        
        return result
            
def main(args=None):
    rclpy.init(args=args)
    tts_action_server = TTSActionServer()
    rclpy.spin(tts_action_server)
    tts_action_server.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()