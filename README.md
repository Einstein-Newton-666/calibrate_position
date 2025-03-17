# calibrate_position
标定相机和imu的相对位置
## 文件结构
```

calibrate_position/
├── CMakeLists.txt 
├── package.xml
├── .gitignore
├── config/
│ └── calibrate_position.yaml 参数配置文件
├── launch/
│ ├──calibrate_position.launch.py 标定程序启动文件
│ └──calibrate_test.launch.py 数学仿真
├── src/
│ ├── calibrate_node.cpp  标定程序源代码
│ └── calibrate_test.cpp  数学仿真程序源代码
└── include/
│ └── calibrate_position/
│   └── calibrate_position.hpp
└── README.md 使用说明
```

## 食用方法
1. 编译好rm_vision及该模块
2. 找一块装甲板
3. 打开config文件夹修改参数
4. 运行下面的指令
```
ros2 launch calibrate_position calibrate_position.launch.py
```

同时晃动云台，确保视野中有且只有一块装甲板(如果出现多块装甲板标定会出问题)

5. 当终端输出"标定结束"，打开标定文件读取参数(默认保存在src/calibrate_position/data.txt文件中)

# 注意
在仿真测试中标定结果经常出现奇怪的问题如正负号相反，姿态多了或少了180°，建议标定后实际测试一下。。。