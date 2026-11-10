# MaixCAM2 视觉端

| 位置 | 对应部分 | 看什么 |
| --- | --- | --- |
| [ball_udp.py](ball_udp.py) | 视觉与坐标发送 | 球检测、位置滤波、触屏目标与 UDP 5005 |

运行在 MaixCAM2；先部署匹配模型，再与 ESP32 加入同一网络。坐标帧为 B,seq,ball_cm,target_cm，丢球帧为 N,seq。
