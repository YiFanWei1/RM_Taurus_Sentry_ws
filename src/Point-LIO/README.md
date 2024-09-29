# 写在前面，这是Taurus自制的升级ROS2的Point-LIO,请勿外传
## 第一次上传
第一次升级ros2大部分内容是将ros1注释了，再另起行写ros2
如有兴趣可以一起来找茬，或者用beyond comapre对比一下
## 主要改动部分
1. ros1的ros相关定义和使用是在main函数，ros2需要封装成LaserMappingNode类才能使用
2. 发布者和订阅者的ros2更改
3. 参数服务器的加载（很奇怪的是有些参数可以加载，有些则不行，这是待解决的问题）
4. 回调函数的share指针更改
5. 时间戳和秒数的转换get_ros_time() get_time_sec()
## 第一次改完后出现的问题
还是会飘，在工训的充电区大概率都会寄