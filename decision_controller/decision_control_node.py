import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import sys
import termios
import tty
import select
import threading
import math



class CmdVelControlNode(Node):

    def __init__(self):
        super().__init__('decision_control_node')

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
        # obstacleトピックをサブスクライブ
        self.obstacle_subscription = self.create_subscription(
            Bool,
            'obstacle_detected',
            self.obstacle_callback,
            10
        )

        self.is_avoidance_area_sub = self.create_subscription(
            Bool,
            'goal_status2',
            self.is_avoidance_toggle,
            10
        )
        self.is_avoidance_area = False
        

        ## Publisher
        # Publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize flags and Twist message
        self.go_flag   = True  # 初期状態をGOに設定
        self.obstacle_detected = False #障害物検出フラグ
        self.current_cmd_vel = Twist()

        # For default stop signal
        self.default_stop_cmd           = Twist()
        self.default_stop_cmd.linear.x  = 0.0
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
        self.get_logger().info('\rNode is running. Press SPACE to toggle GO/NOGO, Q to quit.')

        self.obstacle_timer   = None
        self.ignore_obstacles = False 
        self.ignore_timer     = None

        self.compensate_turn  = False
        self.compensate_timer = None

        self.ignore_twist_cmd = Twist()
        self.ignore_twist_cmd.linear.x = 0.5  # 前進速度
        self.ignore_twist_cmd.angular.z = -math.pi/15  # 右方向への角速度（負の値）

        self.compensate_twist_cmd = Twist()
        self.compensate_twist_cmd.linear.x = 0.5  # 前進速度
        self.compensate_twist_cmd.angular.z = math.pi / 15  # 左回転の角速度

    def monitor_keyboard_input(self):
        """Monitor keyboard input and toggle go_flag on SPACE, P, and Q keys."""
        while rclpy.ok():
            key = self.get_key()
            if   key == ' ':
                self.toggle_go_flag()
            elif key == 'p':
                self.resume_to_next_target()
            elif key == 'q':
                self.shutdown_node()  # Qキーが押されたらノードを終了

    def toggle_go_flag(self):
        """Toggle go_flag and log the state."""
        self.go_flag = not self.go_flag
        state = "GO" if self.go_flag else "NOGO"
        self.get_logger().info(f'State changed to: {state}\r')
    
    def is_avoidance_toggle(self,msg):
        if   msg.data == True :
            self.is_avoidance_area = True
            self.get_logger().info("Resuming to the next target.\r")
        elif msg.data == False:
            self.is_avoidance_area = False
        else :
            return

    def resume_to_next_target(self):
        """Resume movement and update to the next target (stop point or goal)."""
        if getattr(self, 'reached_target_flag', False) or getattr(self, 'reached_obstacle_flag', False):
            self.get_logger().info("Resuming to the next target.\r")
            self.reached_target_flag = False  # ゴールまたは停止ポイント到達フラグをリセット
            self.reached_obstacle_flag = False
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
            if self.compensate_turn:
                cmd_msg = self.compensate_twist_cmd
                self.get_logger().info("もとの経路へ回帰中\r")
            elif self.ignore_obstacles:
                cmd_msg = self.ignore_twist_cmd
                self.get_logger().info("障害物回避動作中\r")
            elif not self.obstacle_detected:
                cmd_msg = self.current_cmd_vel
                self.get_logger().info("通常運転運転中\r")
            else:
                cmd_msg = self.default_stop_cmd 
                self.get_logger().info("障害物検知により一時停止中\r")
        # NOGO状態では停止メッセージ
        else:
            cmd_msg = self.default_stop_cmd 
            self.get_logger().info("一時停止中\r")

        
        self.publisher.publish(cmd_msg)


    def cmd_vel_tmp_callback(self, msg):
        # cmd_vel_tmpの値を受け取り、現在の制御量を更新
        self.current_cmd_vel = msg
        self.publish_cmd_vel()
        
    
    def odom_callback(self, msg):
        """Check proximity to current target and update go_flag accordingly."""
        # 現在のターゲット（停止ポイントまたはゴール）の座標
        target_x, target_y = self.current_target

        # 現在の位置
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.distance_to_goal = math.hypot(target_x - self.current_x, target_y - self.current_y)

        if self.is_avoidance_area :
            self.get_logger().info("回避行動有効エリア\r")
        else :
            self.get_logger().info("回避行動無効エリア：障害物消失まで停止します。\r")
        self.get_logger().info(f"Distance to Target {self.current_target_index + 1}: {self.distance_to_goal:.2f}\r")
        self.get_logger().info(f"Current Position: ({self.current_x:.2f}, {self.current_y:.2f})\r")
        self.get_logger().info(f"Target Position: ( {target_x:.2f},  {target_y:.2f})\r")


        # ターゲット（停止ポイントまたはゴール）に到達した場合
        if self.distance_to_goal < self.distance_threshold:
            if self.current_target == self.goal:
                # ゴールに到達した場合は停止して終了
                self.get_logger().info("ゴールに到達しました。\r")
 
            else:
                # 停止ポイントに到達した場合は一時停止
                self.get_logger().info(f"停止ポイント {self.current_target_index + 1} : {self.stop_points[self.current_target_index]} に到達。NOGO状態に切り替えます。\r")
                self.go_flag = False
                self.reached_target_flag = True


    def obstacle_callback(self, msg):
        """Update obstacle_detected flag based on sensor data."""
        if self.ignore_obstacles or self.compensate_turn:
            return

        if msg.data and not self.obstacle_detected:
            # 障害物が新たに検出された場合
            self.obstacle_detected = True
            self.get_logger().info("障害物を検出しました。NOGO状態に切り替えます。\r")

            if self.obstacle_timer is None and self.is_avoidance_area == True:
                self.obstacle_timer = threading.Timer(10.0, self.reset_obstacle_detected)
                self.obstacle_timer.start()
               
        elif not msg.data and self.obstacle_detected:
            # 障害物がなくなった場合
            self.obstacle_detected = False
            self.get_logger().info("障害物がなくなりました。GO状態に切り替えます。\r")

            if self.obstacle_timer is not None:
                self.obstacle_timer.cancel()
                self.obstacle_timer = None

 
    def reset_obstacle_detected(self):
        """Reset obstacle_detected flag after timer expires."""
        self.obstacle_detected = False
        self.obstacle_timer = None
        self.get_logger().info("10秒が経過しました。回避行動に移ります。\r")
        self.ignore_obstacles = True

        if self.ignore_timer is None:
            self.ignore_timer = threading.Timer(3.0, self.reset_ignore_obstacles)
            self.ignore_timer.start()


    def reset_ignore_obstacles(self):
        """Reset ignore_obstacles flag after timer expires."""
        self.ignore_obstacles = False
        self.ignore_timer = None
        self.get_logger().info("5秒が経過しました。補正します。\r")

        self.compensate_turn = True

        if self.compensate_timer is None:
            self.compensate_timer = threading.Timer(6.0, self.reset_compensate_turn)
            self.compensate_timer.start()


    def reset_compensate_turn(self):
        """Reset compensate_turn flag after timer expires."""
        self.compensate_turn = False
        self.compensate_timer = None
        self.get_logger().info("補正期間が終了しました。通常の動作に戻ります。\r")
        

    # handle keyboad input
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


    # shutdown node
    def shutdown_node(self):
        """Shutdown the node and stop publishing to /cmd_vel."""
        self.get_logger().info('Shutting down the node.\r')
        self.publisher.publish(self.default_stop_cmd)

        if self.obstacle_timer is not None:
            self.obstacle_timer.cancel()
            self.obstacle_timer = None
        if self.ignore_timer is not None:
            self.ignore_timer.cancel()
            self.ignore_timer = None
        if self.compensate_timer is not None:
            self.compensate_timer.cancel()
            self.compensate_timer = None

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
