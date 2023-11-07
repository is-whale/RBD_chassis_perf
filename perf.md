### 优化记录

1.不同帧的CAN信息通过循环中的不同函数处理。

2.CAN消息传递延时
CMD -> can_frame -> mux_can_fountion->sent_messages->can_frame

PS:

rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" 