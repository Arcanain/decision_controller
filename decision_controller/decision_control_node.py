import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped 
import sys
import termios
import tty
import select
import threading
import math


class CmdVelControlNode(Node):

    def __init__(self):
        super().__init__('decision_control_node')

        # =========================
        # Subscribers
        # =========================
        # /cmd_vel_tmp: ローカルプランナなどからの元の速度指令
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel_tmp',
            self.cmd_vel_tmp_callback,
            20
        )

        # /odom: 自己位置
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # GNSS Pose: スイッチポイント監視用（座標系が odom と異なる）
        self.gnss_subscription = self.create_subscription(
            PoseStamped,          # ★ 実際のメッセージ型に合わせて変更
            'gnss_pose',          # ★ 実際のトピック名に合わせて変更
            self.gnss_callback,   # ★ 新しく作るコールバック
            10
        )


        # /obstacle_detected: 障害物検知（True = 障害物あり）
        self.obstacle_subscription = self.create_subscription(
            Bool,
            'obstacle_detected',
            self.obstacle_callback,
            10
        )

        # /dwa_active: DWA が現在有効かどうか
        self.dwa_active_subscription = self.create_subscription(
            Bool,
            'dwa_active',
            self.dwa_active_callback,
            10
        )

        # =========================
        # Publisher
        # =========================
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        ## TODO ゴール到達も見る

        # =========================
        # 基本状態フラグ
        # =========================
        self.go_flag = True                # SPACE で GO/NOGO 切り替え
        self.current_cmd_vel = Twist()     # /cmd_vel_tmp からの現在コマンド

        # デフォルトの停止コマンド
        self.default_stop_cmd = Twist()
        self.default_stop_cmd.linear.x = 0.0
        self.default_stop_cmd.angular.z = 0.0

        # =========================
        # 停止ポイント / ゴール
        # =========================
        self.stop_points = [
            (26.6831,  30.5581),  # 横断歩道1
            (63.6472,  35.7471),  # 横断歩道2
            (140.067,  29.0186),  # 横断歩道3
            (223.646, -68.5382),  # 白線1
            (222.879, -75.2085),  # 点字1
            (221.272, -95.0025),  # 白線2
            (224.8,   -97.1773),  # 点字2
            ]   
        
        self.distance_threshold = 4.5
        self.current_stop_point_index = 0
        self.current_target = self.stop_points[self.current_stop_point_index]

        self.goal = (-50.0, -50.0)   # 最終ゴール

        # 停止ポイント到達フラグ
        self.reached_target_flag = False
        
        
        self.switch_points = [
            (-2.170683,36.495309),
            (36.083191,140.076971),    # GNSS→EMCL ①
            #(70.0, 40.0),    # GNSS→EMCL ②
        ]
        self.current_switch_point_index = 0
        self.current_switch_point = self.switch_points[self.current_switch_point_index] 
        
        # リスト，Switch Pointの一つ目に到達したら一度だけδTrueをpublishする
        self.goal_pub_1 = self.create_publisher(Bool, 'gnss_emcl_1', 10)
        # 二つ目に到達したら一度だけTrueをpublishする
        self.goal_pub_2 = self.create_publisher(Bool, 'gnss_emcl_2', 10)
        
        # =========================
        # 障害物処理
        # =========================
        # センサ値そのもの（True = 障害物あり）※obstacle_callbackで更新するだけ
        self.obstacle_raw = False

        # 「今、停止すべきかどうか」を表す内部フラグ（Timer で管理）
        self.obstacle_detected = False

        # センサが True になった時刻（継続している間は同じ障害物扱い）
        self.obstacle_start_time = None

        # ① 障害物検知 → 即停止
        # ② obstacle_stop_duration 秒間は停止
        # ③ それを過ぎたら「一度だけ」再開（同じ連続検知に対しては再度止まらない）
        self.obstacle_stop_duration = 10.0  # [sec]

        # 障害物の状態を周期的に監視する Timer（例: 10Hz）
        self.obstacle_check_timer = self.create_timer(
            0.1,
            self.update_obstacle_state
        )

        # =========================
        # DWA フラグ（タイムスタンプ方式）
        # =========================
        self.dwa_active = False
        self.dwa_timeout = 1.0  # [sec] この時間メッセージが来なければ OFF 扱い
        self.last_dwa_time = self.get_clock().now()

        # 周期的に DWA のタイムアウトを監視する Timer（10Hz）
        self.dwa_check_timer = self.create_timer(
            0.1,
            self.check_dwa_timeout
        )

        # =========================
        # キーボード入力スレッド
        # =========================
        self.input_thread = threading.Thread(target=self.monitor_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        self.get_logger().info(
            'Node is running. Press SPACE to toggle GO/NOGO, '
            'P to go to next target, Q to quit.\r'
        )

    # ============================================================
    # Keyboard handling
    # ============================================================
    def monitor_keyboard_input(self):
        """キーボード入力を監視し、SPACE / P / Q に応じて操作。"""
        while rclpy.ok():
            key = self.get_key()
            if key is None:
                continue

            if key == ' ':
                self.toggle_go_flag()
            elif key == 'p':
                self.resume_to_next_target()
            elif key == 'q':
                self.shutdown_node()
                break

    def toggle_go_flag(self):
        """GO/NOGO をトグル。"""
        self.go_flag = not self.go_flag
        state = "GO" if self.go_flag else "NOGO"
        self.get_logger().info(f'State changed to: {state}\r')
        self.publish_cmd_vel()

    def get_key(self):
        """非ブロッキングで1文字だけキーボード入力を取得。"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                return key.lower()
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # ============================================================
    # Target management
    # ============================================================
    def resume_to_next_target(self):
        """停止ポイント到達後、次のターゲット（停止ポイント or ゴール）へ進む。"""
        if not self.reached_target_flag:
            self.get_logger().info("No reached target flag. Ignore 'p' key.\r")
            return

        self.get_logger().info("Resuming to the next target.\r")
        self.reached_target_flag = False
        self.go_flag = True

        # 次の停止ポイント or ゴールへ
        self.current_stop_point_index += 1
        if self.current_stop_point_index < len(self.stop_points):
            self.current_target = self.stop_points[self.current_stop_point_index]
            self.get_logger().info(
                f"Next stop point: {self.current_stop_point_index + 1} at {self.current_target}\r"
            )
        else:
            self.current_target = self.goal
            self.get_logger().info(
                f"All stop points passed. Heading to goal: {self.goal}\r"
            )

        self.publish_cmd_vel()

    # ============================================================
    # cmd_vel publishing
    # ============================================================
    def publish_cmd_vel(self):
        """最終的な /cmd_vel を出力する。"""
        if not self.go_flag:
            # NOGO のときは必ず停止
            cmd_msg = self.default_stop_cmd
            self.get_logger().info("NOGO: stop\r")
        # DWA ON の間は障害物停止を無視する
        elif self.obstacle_detected and not self.dwa_active:
            cmd_msg = self.default_stop_cmd
            self.get_logger().info("Obstacle detected (DWA OFF): stop\r")
        else:
            cmd_msg = self.current_cmd_vel
            if self.dwa_active:
                self.get_logger().info("DWA ON: ignore obstacle and run normally\r")
            else:
                self.get_logger().info("Normal operation\r")

        self.publisher.publish(cmd_msg)

    # ============================================================
    # Callbacks
    # ============================================================
    def cmd_vel_tmp_callback(self, msg: Twist):
        """/cmd_vel_tmp を受け取り、状態に応じて /cmd_vel を出す。"""
        self.current_cmd_vel = msg
        self.publish_cmd_vel()

    def odom_callback(self, msg: Odometry):
        """
        停止ポイント / ゴール / スイッチポイント までの距離を監視する。
        - 停止ポイント到達 → NOGO にして一時停止（resume_to_next_target で再開）
        - ゴール到達       → NOGO にして停止（必要ならここで処理を追加）
        - スイッチポイント到達 → 一度だけ gnss_emcl_1 / gnss_emcl_2 に True を publish
        """
        # 現在位置
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # =========================
        # 停止ポイント / ゴールの距離チェック
        # =========================
        target_x, target_y = self.current_target
        distance_to_target = math.hypot(target_x - current_x, target_y - current_y)

        self.get_logger().info(
            f"Distance to Target {self.current_stop_point_index + 1}: {distance_to_target:.2f}\r"
        )
        self.get_logger().info(
            f"Current Position: ({current_x:.2f}, {current_y:.2f}), "
            f"Target: ({target_x:.2f}, {target_y:.2f})\r"
        )

        if distance_to_target < self.distance_threshold:
            if self.current_target == self.goal:
                # ゴール到達
                self.get_logger().info("ゴールに到達しました。NOGO状態に切り替えます。\r")
                self.go_flag = False
                self.reached_target_flag = False  # ゴールは「次へ進む」対象ではない想定
                self.publish_cmd_vel()
            else:
                # 停止ポイント到達 → NOGO に切り替え
                self.get_logger().info(
                    f"停止ポイント {self.current_stop_point_index + 1} : "
                    f"{self.stop_points[self.current_stop_point_index]} に到達。"
                    "NOGO状態に切り替えます。\r"
                )
                self.go_flag = False
                self.reached_target_flag = True
                self.publish_cmd_vel()

        # =========================
        # スイッチポイントの距離チェック
        # =========================
        self.get_logger().info(
            f"current_switch_point_index {self.current_switch_point_index}, "
            f"len(self.switch_points) {self.switch_points}: \r"
        )
        if self.current_switch_point_index < len(self.switch_points):
            sx, sy = self.current_switch_point
            distance_to_switch = math.hypot(sx - current_x, sy - current_y)

            self.get_logger().info(
                f"Distance to Switch Point by ODOM{self.current_switch_point_index + 1}: "
                f"{distance_to_switch:.2f}\r"
                f"x={sx},y={sy}"
                f"Current Position: ({current_x:.2f}, {current_y:.2f})"
            )

            if distance_to_switch < self.distance_threshold:
                # 一度だけ True を publish
                msg_bool = Bool()
                msg_bool.data = True

                if self.current_switch_point_index == 0:
                    self.goal_pub_1.publish(msg_bool)
                    self.get_logger().info(
                        "Switch point 1 reached: published gnss_emcl_1 = True\r"
                    )
                elif self.current_switch_point_index == 1:
                    self.goal_pub_2.publish(msg_bool)
                    self.get_logger().info(
                        "Switch point 2 reached: published gnss_emcl_2 = True\r"
                    )

                # 同じポイントで何度も publish しないように index を進める
                self.current_switch_point_index += 1
                if self.current_switch_point_index < len(self.switch_points):
                    self.current_switch_point = \
                        self.switch_points[self.current_switch_point_index]
                        
                        

    def gnss_callback(self, msg: PoseStamped):
        """
        GNSS座標系での位置を監視し、switch_points への接近を判定するコールバック。
        座標系は switch_points と同じ前提（GNSS 座標系）。

        - switch_points[0] に到達したら gnss_emcl_1 に True を一度だけ publish
        - switch_points[1] に到達したら gnss_emcl_2 に True を一度だけ publish
        """
        # GNSS での現在位置
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y

        # もうすべてのスイッチポイントを処理し終わっていたら何もしない
        if self.current_switch_point_index >= len(self.switch_points):
            return

        sx, sy = self.current_switch_point
        distance_to_switch = math.hypot(sx - current_x, sy - current_y)


        self.get_logger().info(
            f"[GNSS] Distance to Switch Point {self.current_switch_point_index + 1}: "
            f"{distance_to_switch:.2f}\r"
        )

        if distance_to_switch < self.distance_threshold:
            # 一度だけ True を publish
            msg_bool = Bool()
            msg_bool.data = True

            if self.current_switch_point_index == 0:
                #self.goal_pub_1.publish(msg_bool)
                self.get_logger().info(
                    "Switch point 1 reached (GNSS): published gnss_emcl_1 = True\r"
                )
            elif self.current_switch_point_index == 1:
                #self.goal_pub_2.publish(msg_bool)
                self.get_logger().info(
                    "Switch point 2 reached (GNSS): published gnss_emcl_2 = True\r"
                )

            # 同じポイントで何度も publish しないように index を進める
            self.current_switch_point_index += 1
            if self.current_switch_point_index < len(self.switch_points):
                self.current_switch_point = \
                    self.switch_points[self.current_switch_point_index]
    
    
    
    def obstacle_callback(self, msg: Bool):
        """
        センサ値を内部変数に書き込むだけ。
        停止/再開の判断は update_obstacle_state() の Timer 側で行う。

        - dwa_active が True のときは「障害物処理そのものを無視」するので、
          センサ値も更新しない。
        """
        if self.dwa_active:
            # DWA 有効中は障害物を完全に無視（内部状態も変えない）
            return

        # DWA OFF のときのみ生のセンサ値を更新
        self.obstacle_raw = msg.data

    def update_obstacle_state(self):
        """
        Timer コールバック。
        obstacle_raw と経過時間にもとづき、
        obstacle_detected（「止まるべきか」）を管理する。

        仕様:
          - obstacle_raw が False → obstacle_detected = False（停止指令解除）
          - obstacle_raw が False→True になった瞬間:
                obstacle_start_time を記録し、obstacle_detected = True（即停止）
          - obstacle_raw が True のまま:
                経過時間 < obstacle_stop_duration の間は停止継続
                経過時間 >= obstacle_stop_duration になったら一度だけ
                obstacle_detected = False にして進ませる
                （同じ連続 True に対して再度 True にはしない）
        """
        now = self.get_clock().now()

        if not self.obstacle_raw:
            # 障害物なし
            if self.obstacle_detected or self.obstacle_start_time is not None:
                # 状態が変化したときだけログ＆publish
                self.obstacle_detected = False
                self.obstacle_start_time = None
                self.get_logger().info("Obstacle state: clear. Resume if other conditions allow.\r")
                self.publish_cmd_vel()
            return

        # ここから obstacle_raw == True (障害物あり)
        if self.obstacle_start_time is None:
            # 立ち上がりエッジ：新しい障害物とみなす
            self.obstacle_start_time = now
            self.obstacle_detected = True
            self.get_logger().info("Obstacle detected: start stopping period.\r")
            self.publish_cmd_vel()
            return

        # 継続検知中：経過時間で制御
        elapsed = (now - self.obstacle_start_time).nanoseconds * 1e-9

        if elapsed < self.obstacle_stop_duration:
            # 停止継続
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.get_logger().info(
                    f"Obstacle still present (<{self.obstacle_stop_duration}s). Keep stopping.\r"
                )
                self.publish_cmd_vel()
        else:
            # 規定時間経過後：一度だけ再開を試みる
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.get_logger().info(
                    f"Obstacle present for {elapsed:.2f}s >= {self.obstacle_stop_duration}s. "
                    "Ignore once and try to move.\r"
                )
                self.publish_cmd_vel()
            # その後は obstacle_raw が False になるまで、
            # 同じ障害物に対しては「再度止まらない」

    def dwa_active_callback(self, msg: Bool):
        """
        DWA からのフラグが来るたびに現在時刻を更新。
        True が定期的に来ている間だけ self.dwa_active = True とし、
        一定時間来なければ自動で OFF に戻す。
        """
        now = self.get_clock().now()
        self.last_dwa_time = now  # タイムスタンプ更新

        self.dwa_active = msg.data
        state = "ON" if self.dwa_active else "OFF"
        self.get_logger().info(f"DWA callback received: {state}\r")

    def check_dwa_timeout(self):
        """最後に受信した DWA 時刻からの経過時間を調べ、タイムアウトしたら OFF にする。"""
        now = self.get_clock().now()
        dt = (now - self.last_dwa_time).nanoseconds * 1e-9  # 秒に変換

        if dt > self.dwa_timeout:
            if self.dwa_active:
                self.get_logger().info(
                    f"DWA timeout ({dt:.2f}s). Setting DWA OFF.\r"
                )
            self.dwa_active = False

    # ============================================================
    # Shutdown
    # ============================================================
    def shutdown_node(self):
        """ノードをシャットダウンし、/cmd_vel を停止させる。"""
        self.get_logger().info('Shutting down the node.\r')
        self.publisher.publish(self.default_stop_cmd)

        # rclpy の Timer はノード破棄時に自動で止まる

        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_node()


if __name__ == '__main__':
    main()
