# 取得完了時刻。各値は順番に取得するため、同時刻の測定値ではありません。
std_msgs/Header header

# 電流実測値 [mA]
int32 current_ma

# 速度実測値。単位はEPOS2の速度単位設定に従います（rpmとは限りません）。
int32 velocity_actual

# 位置実測値 [quadcounts。通常エンコーダ1周期あたり4カウント]
int32 position_counts

# 出力軸速度 = velocity_actual / gear_ratio。単位はvelocity_actualと同じです。
float64 output_shaft_velocity

# 出力軸位置 = position_counts / gear_ratio。単位はモータ軸換算のquadcountです。
float64 output_shaft_position_motor_counts

# 出力軸の推定位置 [rad]。EPOS2の位置基準に依存し、絶対角度ではありません。
float64 output_shaft_position_rad
