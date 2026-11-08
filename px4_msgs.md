# PX4_msgs
## 问题
### 1.编译问题
1.1 AttributeError: module 'em' has no attribute 'BUFFERED_OPT', 最新的empy==4.x与ROS2 Humble不兼容，需要降低  
```
python -m pip install empy==3.3.4
python -m pip show empy # 看一下安装到哪里了
which empy
empy --version
python -c "import em; print(em.__file__); print(hasattr(em, 'BUFFERED_OPT'))" # 检查报错点
```

1.2缺少python包

ModuleNotFoundError: No module named '**catkin_pkg**'  
ModuleNotFoundError: No module named '**lark**'

`python -m pip install catkin_pkg`
