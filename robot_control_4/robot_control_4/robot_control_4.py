#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv
import datetime

class MoveAndTurnNode(Node):
    """
    SCOUT MINIが「3m前進 -> 180度時計回り」を実行し、
    その間のオドメトリデータをCSVに記録するノード。
    """
    def __init__(self):
        super().__init__('move_and_turn_node')
        
        # ▼▼▼ 変更点: パラメータと状態の定義 ▼▼▼
        # パラメータ（定数）
        self.target_distance_ = 3.0  # 1辺の目標距離 (m)
        self.target_angle_ = -math.pi # 目標回転角度 (rad) (-180度)
        self.rotation_tolerance_ = 0.2 # 回転停止の許容誤差 (rad, 約11.5度)
        
        self.linear_velocity_ = 0.3    # 前進速度 (m/s)
        self.angular_velocity_ = 0.3   # 回転速度 (rad/s) (正の値)
        
        self.odom_topic_ = '/odom'
        self.cmd_vel_topic_ = '/cmd_vel'
        
        # 状態定義
        self.STATE_INIT = "INIT"
        self.STATE_MOVING_FORWARD = "MOVING_FORWARD"
        self.STATE_ROTATING = "ROTATING"
        self.STATE_DONE = "DONE"
        
        # 状態変数
        self.current_state_ = self.STATE_INIT
        self.initial_position_ = None
        self.current_position_ = None
        self.current_yaw_ = 0.0
        self.distance_traveled_ = 0.0
        self.angle_traveled_ = 0.0
        
        self.last_yaw_ = 0.0 # 累積回転角の計算用
        
        self.timer_ = None             # timer_callback用タイマー
        self.init_timer_ = None        # 起動遅延用タイマー
        # ▲▲▲ 変更点 ▲▲▲

        # CSV保存用の変数初期化
        self.csv_file_ = None
        self.csv_writer_ = None
        self.csv_filename_ = ""
        try:
            now = datetime.datetime.now()
            self.csv_filename_ = f'odom_log_{now.strftime("%Y%M%d_%H%M%S")}.csv'
            self.csv_file_ = open(self.csv_filename_, 'w', newline='', encoding='utf-8')
            self.csv_writer_ = csv.writer(self.csv_file_)
            self.csv_writer_.writerow(['timestamp', 'x', 'y', 'z', 'yaw'])
            self.get_logger().info(f'オドメトリデータを [{self.csv_filename_}] に保存します。')
        except IOError as e:
            self.get_logger().error(f'CSVファイルのオープンに失敗しました: {e}')
            self.csv_file_ = None
            self.csv_writer_ = None

        # パブリッシャーとサブスクライバー
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic_, 10)
        self.subscription_ = self.create_subscription(
            Odometry,
            self.odom_topic_,
            self.odom_callback,
            10)
        
        # ▼▼▼ 変更点: ログメッセージ ▼▼▼
        self.get_logger().info(
            f'ノード起動: [3m前進 -> 180度回転] を実行します...'
        )
        # ▲▲▲ 変更点 ▲▲▲
        self.get_logger().info(f'オドメトリトピック [{self.odom_topic_}] を待機中...')

        # ドライバの初期化（error_code: 4 回避）のため、3秒待ってから制御ループを開始
        self.get_logger().warn('Scoutドライバの初期化を待機中... (3秒)')
        self.init_timer_ = self.create_timer(3.0, self.start_control_loop)

    def start_control_loop(self):
        """
        3秒後に呼び出され、メインの制御ループタイマーを開始する。
        """
        if self.init_timer_:
            self.init_timer_.cancel()
            self.init_timer_ = None
        
        self.get_logger().info('待機完了。制御ループを開始します。')
        
        # 最初の状態に遷移
        self.current_state_ = self.STATE_MOVING_FORWARD
        self.get_logger().info(f'状態遷移: {self.current_state_}')
        
        # 制御ループタイマー (50Hz, 0.02秒)
        self.timer_ = self.create_timer(0.02, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        """
        オドメトリデータを受信したときのコールバック
        (ロジックは MoveSquareNode と同一です)
        """
        self.current_position_ = msg.pose.pose.position
        
        # クォータニオンからヨー角を計算
        q = msg.pose.pose.orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw_ = math.atan2(t3, t4)

        if self.csv_writer_:
            try:
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.csv_writer_.writerow([
                    timestamp, 
                    self.current_position_.x, self.current_position_.y, self.current_position_.z,
                    self.current_yaw_
                ])
            except Exception as e:
                self.get_logger().error(f"CSV書き込みエラー: {e}", throttle_duration_sec=10)
        
        # 最初の位置と向きを記録
        if self.initial_position_ is None:
            self.initial_position_ = self.current_position_
            self.last_yaw_ = self.current_yaw_ # 累積計算のため、last_yaw も初期化
            self.get_logger().info(
                f'初期位置・ヨーを記録: x={self.initial_position_.x:.2f}, y={self.initial_position_.y:.2f}, yaw={self.current_yaw_:.2f}'
            )

        # 累積回転角を計算
        if self.initial_position_ is not None:
            # 回転状態の時だけ、回転角度を累積する
            if self.current_state_ == self.STATE_ROTATING:
                diff_yaw = self.current_yaw_ - self.last_yaw_
                
                # ヨー角のラップアラウンド（-π <-> +π）を考慮
                if diff_yaw > math.pi:
                    diff_yaw -= 2 * math.pi
                elif diff_yaw < -math.pi:
                    diff_yaw += 2 * math.pi
                
                self.angle_traveled_ += diff_yaw
            
            self.last_yaw_ = self.current_yaw_


    def timer_callback(self):
        """
        制御ループ (ステートマシン)。50Hzで実行されます。
        """
        if self.initial_position_ is None:
            self.get_logger().warn('まだオドメトリデータを受信していません...', throttle_duration_sec=5)
            return

        twist_msg = Twist()

        # ▼▼▼ 変更点: ステートマシンを単純化 ▼▼▼

        # --- 状態 1: 3m前進 ---
        if self.current_state_ == self.STATE_MOVING_FORWARD:
            # 初期位置からの移動距離を計算
            dx = self.current_position_.x - self.initial_position_.x
            dy = self.current_position_.y - self.initial_position_.y
            self.distance_traveled_ = math.sqrt(dx*dx + dy*dy)

            if self.distance_traveled_ < self.target_distance_:
                # 目標未達: 前進
                twist_msg.linear.x = self.linear_velocity_
                self.get_logger().info(
                    f'前進中: {self.distance_traveled_:.2f}m / {self.target_distance_}m',
                    throttle_duration_sec=1
                )
            else:
                # 目標達成: 停止し、回転状態へ
                self.get_logger().info(f'目標距離 {self.target_distance_}m に到達。')
                self.current_state_ = self.STATE_ROTATING
                self.get_logger().info('状態遷移: 回転 (ROTATING)')
                # 回転のための初期値をリセット
                self.angle_traveled_ = 0.0
                # self.last_yaw_ は odom_callback で常に更新されているのでOK

        # --- 状態 2: 180度時計回り ---
        elif self.current_state_ == self.STATE_ROTATING:
            # self.angle_traveled_ は odom_callback で累積計算されている
            
            # (目標角度 - 許容誤差) よりもまだ大きいか？ 
            # (例: -3.0 > (-3.14 + 0.2))
            if self.angle_traveled_ > (self.target_angle_ + self.rotation_tolerance_):
                # まだ回り足りない: 時計回りに回転
                twist_msg.angular.z = -self.angular_velocity_ # 負のZが時計回り
                self.get_logger().info(
                    f'回転中: {self.angle_traveled_:.2f}rad / {self.target_angle_:.2f}rad',
                    throttle_duration_sec=1
                )
            else:
                # 目標達成: 停止し、完了状態へ
                self.get_logger().info(f'目標角度 {self.target_angle_:.2f}rad に到達。')
                self.current_state_ = self.STATE_DONE
                self.get_logger().info('状態遷移: 完了 (DONE)')
                self.get_logger().info('全てのシーケンスが完了しました。')

        # --- 状態 3: 完了 ---
        elif self.current_state_ == self.STATE_DONE:
            # 完了後は停止コマンドを送り続ける
            pass # twist_msg は(0,0,0)で初期化されている
        
        # ▲▲▲ 変更点 ▲▲▲

        # 最終的に計算された速度指令をパブリッシュ
        self.publisher_.publish(twist_msg)

    def close_csv_file(self):
        """
        ノード終了時にCSVファイルをクローズする
        """
        if self.csv_file_:
            self.csv_file_.close()
            self.get_logger().info(f'CSVファイル [{self.csv_filename_}] をクローズしました。')

def main(args=None):
    rclpy.init(args=args)
    # ▼▼▼ 変更点: ノードクラス名を変更 ▼▼▼
    node = MoveAndTurnNode()
    # ▲▲▲ 変更点 ▲▲▲
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('キーボード割り込みによりシャットダウンします...')
    finally:
        # シャットダウン時に必ず停止コマンドを送信する
        if rclpy.ok():
            node.get_logger().info('最後に停止コマンドを送信します。')
            node.publisher_.publish(Twist()) # 停止コマンド
            
            # CSVファイルをクローズする
            node.close_csv_file()
            
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()