#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import csv             # ▼▼▼ 変更点: CSVライブラリをインポート ▼▼▼
import datetime        # ▼▼▼ 変更点: datetimeライブラリをインポート ▼▼▼

class MoveForwardNode(Node):
    """
    SCOUT MINIを指定された距離だけ前進させ、その間のオドメトリデータをCSVに記録するノード。
    """
    def __init__(self):
        super().__init__('move_forward_node')
        
        # パラメータ（定数）
        self.target_distance_ = 3.0  # 目標距離 (メートル)
        self.linear_velocity_ = 0.3   # 前進速度 (メートル/秒)
        self.odom_topic_ = '/odom'     # オドメトリのトピック名
        self.cmd_vel_topic_ = '/cmd_vel' # 速度指令のトピック名
        
        # 状態変数
        self.initial_position_ = None
        self.current_position_ = None
        self.distance_traveled_ = 0.0
        self.goal_reached_ = False
        self.timer_ = None           # timer_callback用タイマー
        self.init_timer_ = None      # 起動遅延用タイマー

        # ▼▼▼ 変更点: CSV保存用の変数を初期化 ▼▼▼
        self.csv_file_ = None
        self.csv_writer_ = None
        self.csv_filename_ = ""
        try:
            # タイムスタンプ付きのファイル名を生成
            now = datetime.datetime.now()
            self.csv_filename_ = f'odom_log_{now.strftime("%Y%m%d_%H%M%S")}.csv'
            
            # ファイルを開き、ライターを準備
            # newline='' はCSV書き込み時の標準的なお作法です
            self.csv_file_ = open(self.csv_filename_, 'w', newline='', encoding='utf-8')
            self.csv_writer_ = csv.writer(self.csv_file_)
            
            # ヘッダーを書き込む
            self.csv_writer_.writerow(['timestamp', 'x', 'y', 'z'])
            
            self.get_logger().info(f'オドメトリデータを [{self.csv_filename_}] に保存します。')

        except IOError as e:
            self.get_logger().error(f'CSVファイルのオープンに失敗しました: {e}')
            self.csv_file_ = None
            self.csv_writer_ = None
        # ▲▲▲ 変更点 ▲▲▲

        # パブリッシャーとサブスクライバー
        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic_, 10)
        self.subscription_ = self.create_subscription(
            Odometry,
            self.odom_topic_,
            self.odom_callback,
            10)
        
        self.get_logger().info(
            f'ノード起動: {self.target_distance_}m 前進します...'
        )
        self.get_logger().info(f'オドメトリトピック [{self.odom_topic_}] を待機中...')

        # ドライバの初期化（error_code: 4 回避）のため、3秒待ってから制御ループを開始
        self.get_logger().warn('Scoutドライバの初期化を待機中... (3秒)')
        self.init_timer_ = self.create_timer(3.0, self.start_control_loop)

    def start_control_loop(self):
        """
        3秒後に呼び出され、メインの制御ループタイマーを開始する。
        """
        # この関数は一度しか呼ばれたくないので、起動用タイマーを破棄する
        if self.init_timer_:
            self.init_timer_.cancel()
            self.init_timer_ = None
        
        self.get_logger().info('待機完了。制御ループを開始します。')
        
        # 制御ループタイマー (50Hz)
        self.timer_ = self.create_timer(0.02, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        """
        オドメトリデータを受信したときのコールバック
        """
        self.current_position_ = msg.pose.pose.position
        
        # ▼▼▼ 変更点: CSV書き込み処理 ▼▼▼
        if self.csv_writer_:
            try:
                # タイムスタンプ (秒 + ナノ秒を秒単位に変換)
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                
                # データを書き込む
                self.csv_writer_.writerow([
                    timestamp, 
                    self.current_position_.x,
                    self.current_position_.y,
                    self.current_position_.z  # Z座標も記録（通常は0に近いですが）
                ])
            except Exception as e:
                # 書き込みエラーが発生した場合（例: ディスクフルなど）
                self.get_logger().error(f"CSV書き込みエラー: {e}", throttle_duration_sec=10)
        # ▲▲▲ 変更点 ▲▲▲
        
        # 最初の位置を記録
        if self.initial_position_ is None:
            self.initial_position_ = self.current_position_
            self.get_logger().info(
                f'初期位置を記録: x={self.initial_position_.x:.2f}, y={self.initial_position_.y:.2f}'
            )

    def timer_callback(self):
        """
        制御ループ。10Hzで実行されます。
        """
        # オドメトリをまだ受信していないか、目標達成済みの場合は何もしない
        if self.initial_position_ is None:
            self.get_logger().warn('まだオドメトリデータを受信していません...', throttle_duration_sec=5)
            return

        if self.goal_reached_:
            # 目標達成後は停止コマンドを送り続ける
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            return

        # 初期位置からの移動距離（ユークリッド距離）を計算
        dx = self.current_position_.x - self.initial_position_.x
        dy = self.current_position_.y - self.initial_position_.y
        self.distance_traveled_ = math.sqrt(dx*dx + dy*dy)

        # 目標距離に達しているかチェック
        if self.distance_traveled_ < self.target_distance_:
            # 目標未達: 前進コマンドを送信
            move_msg = Twist()
            move_msg.linear.x = self.linear_velocity_
            self.publisher_.publish(move_msg)
            
            self.get_logger().info(
                f'移動中... {self.distance_traveled_:.2f}m / {self.target_distance_}m',
                throttle_duration_sec=1
            )
        else:
            # 目標達成: 停止コマンドを送信し、フラグを立てる
            self.goal_reached_ = True
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            
            self.get_logger().info(
                f'目標距離 {self.target_distance_}m に到達しました。ロボットを停止します。'
            )
            self.get_logger().info('ノードを終了するには Ctrl+C を押してください。')

    # ▼▼▼ 変更点: CSVファイルを閉じるためのメソッドを追加 ▼▼▼
    def close_csv_file(self):
        """
        ノード終了時にCSVファイルをクローズする
        """
        if self.csv_file_:
            self.csv_file_.close()
            self.get_logger().info(f'CSVファイル [{self.csv_filename_}] をクローズしました。')
    # ▲▲▲ 変更点 ▲▲▲

def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('キーボード割り込みによりシャットダウンします...')
    finally:
        # シャットダウン時に必ず停止コマンドを送信する
        if rclpy.ok():
            node.get_logger().info('最後に停止コマンドを送信します。')
            node.publisher_.publish(Twist()) # 停止コマンド
            
            # ▼▼▼ 変更点: CSVファイルをクローズする処理を呼び出す ▼▼▼
            node.close_csv_file()
            # ▲▲▲ 変更点 ▲▲▲
            
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
