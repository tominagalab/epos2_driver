# 取得完了時刻。各値は順番に取得するため、同時刻の測定値ではありません。
std_msgs/Header header

# 電流実測値 [mA]
int32 current_ma

# 速度実測値。単位はEPOS2の速度単位設定に従います（rpmとは限りません）。
int32 velocity_actual

# 位置実測値 [quadcounts。通常エンコーダ1周期あたり4カウント]
int32 position_counts
