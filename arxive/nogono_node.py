import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import sys
import termios
import tty
import threading
import select 



class CmdVelControlNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_control_node')
        
        # # Subscriber
        # cmd_vel_tmpをサブスクライブ
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel_tmp',
            self.cmd_vel_tmp_callback,  # callback func
            20
        )
        
        # odomトピックをサブスクライブ
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        '''
        # 赤信号検知トピックをサブスクライブ
        self.red_light_subscription = self.create_subscription(
            Bool,
            'red_light_detected',
            self.red_light_callback,
            10
        )

        # 停止白線検知トピックをサブスクライブ
        self.stop_line_subscription = self.create_subscription(
            Bool,
            'stop_line_detected',
            self.stop_line_callback,
            10
        )
        '''

        # # Publisher
        # cmd_velをパブリッシュ
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # For default stop signal
        default_stop_cmd           = Twist()
        default_stop_cmd.linear.x   = 0.0
        default_stop_cmd.angular.z = 0.0

        # 状態フラグを初期化
        self.go_flag = True
        self.current_cmd_vel = Twist()  # 初期化された制御量

        # 停止位置や閾値の設定（例としてゴールの座標を指定）
        self.target_x, self.target_y = 5.0, 5.0  # 任意のゴール座標
        self.distance_threshold = 0.5

        # TODO: load scv file that has (x,y) of stop point 
        # キーボード入力監視用のスレッド
        #self.get_logger().info('スペースキーで制御を切り替えます (go/NOGO)')
        #self.run_keyboard_listener()
        # スレッドでキーボード入力監視を開始
        self.get_logger().info('スペースキーで制御を切り替えます (go/NOGO)')
        self.keyboard_thread = threading.Thread(target=self.run_keyboard_listener)
        self.keyboard_thread.daemon = True  # プログラム終了時に自動で終了
        self.keyboard_thread.start()

    def run_keyboard_listener(self):
        # キーボードからのスペースキー入力を監視し、状態を切り替える
        while rclpy.ok():
            if self.is_key_pressed():
                key = self.get_key()
                if key == ' ':
                    self.toggle_go_flag()

    def is_key_pressed(self):
        # 非ブロッキングでキーボードの入力を確認
        dr, _, _ = select.select([sys.stdin], [], [], 0.5)  # タイムアウトを0.1秒に設定
        return dr != []

    def get_key(self):
        # キーボードからの単一文字入力を取得
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def toggle_go_flag(self):
        # go_flagを反転し、状態をログに出力
        self.go_flag = not self.go_flag
        state = "GO" if self.go_flag else "NOGO"
        self.get_logger().info(f'{state} 状態に切り替えました。')
        self.publish_cmd_vel()


    def cmd_vel_tmp_callback(self, msg):
        # cmd_vel_tmpの値を受け取り、現在の制御量を更新
        print('sub')
        self.current_cmd_vel = msg
        self.publish_cmd_vel()


    def odom_callback(self, msg):
        # TODO : refactoring
        # Odomからロボットの現在位置を取得
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        # ゴールとの距離を計算
        distance_to_goal = math.hypot(self.target_x - current_x, self.target_y - current_y)
        
        # ゴールに近づいている場合はNOGOに変更
        if distance_to_goal < self.distance_threshold:
            self.get_logger().info("ゴールに接近しました。NOGO状態に切り替えます。")
            self.go_flag = False
            self.publish_cmd_vel()

    def red_light_callback(self, msg):
        # 赤信号が検知された場合はNOGOに変更
        if msg.data:
            self.get_logger().info("赤信号検知。NOGO状態に切り替えます。")
            self.go_flag = False
            self.publish_cmd_vel()

    def stop_line_callback(self, msg):
        # 停止用の白線が検知された場合はNOGOに変更
        if msg.data:
            self.get_logger().info("停止白線を検知しました。NOGO状態に切り替えます。")
            self.go_flag = False
            self.publish_cmd_vel()

    def publish_cmd_vel(self):
        # go状態ではcurrent_cmd_velをそのまま、NOGO状態では停止信号をパブリッシュ
        cmd_msg = self.current_cmd_vel if self.go_flag else Twist()  # NOGO状態ではゼロのTwist
        self.publisher.publish(cmd_msg)
        self.get_logger().info(f'cmd_velをパブリッシュしました: {cmd_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
