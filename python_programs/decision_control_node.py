import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import termios
import tty
import select
import threading
import math



class CmdVelControlNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_control_node')

        ## Subscriber
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

        ## Publisher
        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize flags and Twist message
        self.go_flag = True  # 初期状態をGOに設定
        self.current_cmd_vel = Twist()

        # For default stop signal
        self.default_stop_cmd           = Twist()
        self.default_stop_cmd.linear.x   = 0.0
        self.default_stop_cmd.angular.z = 0.0
        
        # 複数の停止ポイント座標とゴール座標の設定
        self.stop_points = [(8.79564, -4.04099), (-23.7192, -100.026)]  # 例として複数の停止ポイント
        self.goal = (-50.0, -50.0)  # ゴール座標を1点のみ設定
        self.distance_threshold = 0.5
        self.current_target_index = 0  # 最初の停止ポイントからスタート

        # 現在のターゲット（停止ポイントまたはゴール）を設定
        self.current_target = self.stop_points[self.current_target_index]

        # キーボード入力監視用のスレッド
        self.input_thread = threading.Thread(target=self.monitor_keyboard_input)
        self.input_thread.daemon = True  # プログラム終了時にスレッドも終了
        self.input_thread.start()

        # 定期的にTwistメッセージをパブリッシュ
        #self.timer = self.create_timer(0.1, self.publish_cmd_vel)
        self.get_logger().info('Node is running. Press SPACE to toggle GO/NOGO, Q to quit.\n\n')

    def monitor_keyboard_input(self):
        """Monitor keyboard input and toggle go_flag on SPACE, P, and Q keys."""
        while rclpy.ok():
            key = self.get_key()
            if key == ' ':
                self.toggle_go_flag()
            elif key == 'p':
                self.resume_to_next_target()
            elif key == 'q':
                self.shutdown_node()  # Qキーが押されたらノードを終了

    def toggle_go_flag(self):
        """Toggle go_flag and log the state."""
        self.go_flag = not self.go_flag
        state = "GO" if self.go_flag else "NOGO"
        self.get_logger().info(f'State changed to: {state}\n')
        # Immediately publish to reflect state change
        #self.publish_cmd_vel()

    def resume_to_next_target(self):
        """Resume movement and update to the next target (stop point or goal)."""
        if self.reached_target_flag:
            self.get_logger().info("Resuming to the next target.")
            self.reached_target_flag = False  # ゴールまたは停止ポイント到達フラグをリセット
            self.go_flag = True

            # 次の停止ポイントまたはゴールに進む
            self.current_target_index += 1
            if self.current_target_index < len(self.stop_points):
                # 次の停止ポイントに進む
                self.current_target = self.stop_points[self.current_target_index]
            else:
                # すべての停止ポイントを通過したらゴールを設定
                self.current_target = self.goal
        #self.publish_cmd_vel()

    def publish_cmd_vel(self):
        """Publish the current_cmd_vel based on go_flag state."""
        if self.go_flag:
            cmd_msg = self.current_cmd_vel
        elif not self.go_flag:
            cmd_msg = self.default_stop_cmd # NOGO状態では停止メッセージ
        self.publisher.publish(cmd_msg)
        #self.get_logger().info(f'cmd_vel published GOGO ARUMI !!!: {cmd_msg}\n')
        #self.get_logger().info(f'cmd_vel published GOGO ARUMI !!!\n')

    def cmd_vel_tmp_callback(self, msg):
        # cmd_vel_tmpの値を受け取り、現在の制御量を更新
        #print('sub')
        self.current_cmd_vel = msg
        self.publish_cmd_vel()
    
    def odom_callback(self, msg):
        """Check proximity to current target and update go_flag accordingly."""
        # 現在のターゲット（停止ポイントまたはゴール）の座標
        target_x, target_y = self.current_target

        # 現在の位置
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        distance_to_goal = math.hypot(target_x - current_x, target_y - current_y)

        print(f"\n\nDistance to Target {self.current_target_index + 1}: {distance_to_goal}")
        print(f"Current Position: ({current_x}, {current_y})")
        print(f"Target Position: ({target_x}, {target_y})")

        # ターゲット（停止ポイントまたはゴール）に到達した場合
        if distance_to_goal < self.distance_threshold:
            if self.current_target == self.goal:
                # ゴールに到達した場合は停止して終了
                self.get_logger().info("ゴールに到達しました。システムを停止します。")
                self.go_flag = False

            else:
                # 停止ポイントに到達した場合は一時停止
                self.get_logger().info(f"停止ポイント {self.current_target_index + 1} に到達。NOGO状態に切り替えます。")
                self.go_flag = False
                self.reached_target_flag = True
    

 
    def get_key(self):
        """Non-blocking keyboard input getter for single character."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0.1)[0]:  # タイムアウトを0.1秒に設定
                key = sys.stdin.read(1)
                return key.lower()
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def shutdown_node(self):
        """Shutdown the node and stop publishing to /cmd_vel."""
        self.get_logger().info('Shutting down the node.')
        self.publisher.publish(self.default_stop_cmd)
        rclpy.shutdown()  # ROS2のシャットダウン
        self.destroy_node()  # ノードの破棄

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
