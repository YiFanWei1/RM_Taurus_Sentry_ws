1、解压/home/wei/code/test/src/Point-LIO/PCD_SAVE/dense_map_2.zip和/home/wei/code/test/src/rm_simulation/pb_rm_simulation/meshes/RMUC2023_world/meshes/rm_env.zip
2、工作空间下sudo chmod 777 sim_nav.sh
3、./sim_nav.sh

ps：代码是老版本的，可以实现基本功能（定位、导航避障）,新代码在运行效率和代码简洁性有提高，不影响正常使用（代码的注释是之前学习留下来的，自己的一些理解可能会有错误 ^^ ）
仿真的效果和实际车辆使用mid360的区别很大，这部分会导致定位飘掉（上实车绝对不会出现。。），所以这份代码仅作为框架，之前写完这份之后我的测试都是在实车进行的，不会出现上面的问题