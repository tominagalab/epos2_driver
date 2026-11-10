from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 起動時に使用する設定ファイル名。ファイルはパッケージの config/ に置きます。
# ノード4・5用: 'config_node4_node5.yaml'、5台用: 'config.yaml'
# 変更後はパッケージをビルドし直してから起動してください。
DEFAULT_CONFIG_FILE = 'config_node4_node5.yaml'


# ros2 launch が呼び出す関数。起動するノードと引数をまとめて返します。
def generate_launch_description():
    return LaunchDescription([
        # config 引数を省略すると、上で選んだ設定ファイルが使用されます。
        # 一時的に別のファイルを使う場合は、起動時に config:=絶対パス を指定できます。
        DeclareLaunchArgument(
            'config',
            # インストールされた epos2_driver パッケージの config/ から探します。
            default_value=PathJoinSubstitution([
                FindPackageShare('epos2_driver'), 'config', DEFAULT_CONFIG_FILE,
            ]),
            description='Absolute path to the EPOS driver ROS parameters YAML file',
        ),
        # 1つのドライバノードで、設定ファイルに列挙されたモータを管理します。
        Node(
            package='epos2_driver',
            executable='epos_driver_node',
            # YAMLの先頭キー epos_driver_node と一致させます。
            name='epos_driver_node',
            # ノードのログを起動した端末に表示します。
            output='screen',
            # config 引数で選択されたYAMLをROS 2パラメータとして読み込みます。
            parameters=[LaunchConfiguration('config')],
        ),
    ])
