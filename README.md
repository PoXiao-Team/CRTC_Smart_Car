# 破晓战队视觉系统开发日志

## 0. 文档说明

- 本分支将记录我们破晓战队参赛小车视觉系统的开发历程，包括一些安装教程、操作指南和一些选择背后的原因

## 1. 我们要做什么

让小车可以通过摄像头识别物品、计算坐标，并且围绕传入的数据做出智能行为。

## 2. 完成进度

| 阶段 | 内容 | 状态 | 说明 |
|---|---|---|---|
| 理论流程 | 视觉方案、ROS2 架构 | ✅ 已完成 | 已跑通软件流程 |
| 虚拟机环境 | Ubuntu22.04 + ROS2 + OpenCV | ✅ 已完成 | 可编译运行 |
| 宿主机环境 | Conda + YOLOv8 + 标注工具 | ✅ 已完成 | 可训练、可推理 |
| 硬件部署 | 摄像头、小车平台 | ⬜ 待硬件 | 尚未开始 |
| 性能测试 | FPS、延迟、识别率 | ⬜ 待测试 | 尚未开始 |

## 3. 系统架构

### 3.1 架构图

### 3.2 代码结构

```text
虚拟机部分
For_the_break_of_down/
├── vm_receiver.py
└── src/
    ├── aim_interfaces/
    │   ├── msg/
    │   │   └── AimInfo.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── armor_detector/
        ├── config/
        │   └── params.yaml
        ├── include/
             └── armor_detector
             └── armor_detector_node.hpp
        ├── launch/
        │   └── armor_detector.launch.py
        ├── src/
        │   └── armor_detector_node.cpp
        ├── CMakeLists.txt
        └── package.xml
宿主机部分
ImageStream
├── best.pt
└── host_infer.py
硬件选择：（暂无）
通信方案：（暂无）
```

### 3.3 为什么虚拟机是对的

- 首先，虚拟机的环境配置和技术栈安装十分方便，而且在满足环境要求的同时不会让你的电脑磁盘变得乱七八糟
- 其次，在虚拟机上跑ROS2生态也比较快捷

### 3.4 为什么宿主机是对的
- 虚拟机虽然可以虚拟化 CPU、内存、磁盘和网卡，但默认很难直接使用宿主机独显的 CUDA 加速，性能上非常受限，对于我的配置来说，在虚拟机上跑YOLO相当痛苦，很难接住摄像头的高清视频流，分辨率稍微高一点就会卡成ppt，掉帧非常严重，很多时候不得已通过写代码强制降低分辨率来保持读取画面的流畅度
- 但是我的YOLO训练用的是手机拍摄的较高清照片，太低的分辨率会导致识别不出来或者识别效果极差，置信度低到不敢想
- 所以我选择把主要的ROS节点放到虚拟机，在宿主机上读取视频画面并且跑通YOLO识别，以ROS2话题的形式把YOLO检测的结果发给虚拟机，在虚拟机中进行PnP解算并且发送小道具坐标，最终效果我认为还是可以的，不枉我在交创中心拍的一百二十多张总共包括约800个能量核心的图片（）
- 虚拟机和宿主机暂时是通过NAT网络互通的，为了保证虚拟机可以和电脑联通，我在虚拟机里临时写定了宿主机的IP，这里以后可能会修改，目前用这个先跑通

## 4. 虚拟机安装与基础操作

### 4.1 安装 VMware17 和 Ubuntu22.04

- VMware17：<https://zhuanlan.zhihu.com/p/1951919938420865019>
- Ubuntu22.04：<https://blog.csdn.net/C_1Y1_T/article/details/130243874>

### 4.2 常用 Linux 指令
- 这里的指令是不全面的，我只保留了最常用的指令

```bash
pwd          # 显示绝对路径
ls           # 查看目录文件
ls -la       # 查看详细列表
cd ~/xxx     # 切换目录
cd ..        # 返回上一级
mkdir xxx    # 创建文件夹
touch xxx    # 创建空白文件
rm xxx       # 删除文件
rm -r xxx    # 删除文件夹
cat xxx      # 查看文件内容
```

一键安装工具：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

## 5. 虚拟机环境配置

### 5.1 基础环境
- 打开终端，输入
```bash
wget http://fishros.com/install -O fishros && . fishros
```
- 再输入密码之后，就可以看到很多选项，手动输入编号，就可以执行对应的操作，非常方便
```bash
RUN Choose Task:[请输入括号内的数字]
---众多工具，等君来用---
[1]:一键安装:ROS(支持ROS和ROS2,树莓派Jetson)
[2]:一键安装:github桌面版(小鱼常用的github客户端)
[3]:一键配置:rosdep(小鱼的rosdepc,又快又好用)
[4]:一键配置:ROS环境(快速更新ROS环境设置,自动生成环境选择)
[5]:一键配置:系统源(更换系统源,支持全版本Ubuntu系统)
[6]:一键安装:nodejs
[7]:一键安装:VsCode开发工具
[8]:一键安装:Docker
[9]:一键安装:Cartographer(内测版v0.1)
[10]:一键安装:微信(可以在Linux上使用的微信)
[77]:测试模式:运行自定义工具测试
[0]:quit
```
- 这里需要分别输入5（更换系统源）、7、1（一定一定要留意安装的版本）、0（退出）
- 配置python源
- 安装g++：

```bash
sudo apt install g++
```
- 安装cmake：
```bash
sudo apt install cmake
```
- 配置python环境：
```bash
python3 -m pip install --user numpy
```

### 5.2 OpenCV 安装
- 安装opencv-python：
```bash
sudo apt-get install python3-opencv
```
- 安装opencv-cpp：
```bash
# 1.安装依赖项
sudo apt update && sudo apt install -y cmake g++ wget unzip
# 2.创建一个opencv_build文件夹并进入
mkdir opencv_build
cd opencv_build
# 3.下载opencv及opencv_contrib源码压缩包并解压
（如果有网络问题可以到别处直接下载文件后自行解压）
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip
# 4.创建构建目录并切换到该目录
mkdir -p build && cd build
# 5.配置并编译
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x -DBUILD_opencv_highgui=ON -DBUILD_opencv_imgcodecs=ON -DBUILD_opencv_videoio=ON -DBUILD_opencv_objdetect=ON
cmake --build .-j$(nproc)
# 6.安装
sudo cmake --install .
```
- 安装视频编解码依赖：
```bash
sudo apt install libavcodec-dev libavformat-dev libswscale-dev
sudo apt install libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
```
- 安装其他依赖：
```bash
sudo apt install libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev libtiff-dev
```
- 安装v4l2依赖（读取硬件设施用）
```bash
sudo apt install v4l-utils libv4l-dev
```
```bash
sudo Idconfig
```

### 5.3 Git 安装与配置
安装git：
```bash
sudo apt install git
```
- 配置账户信息
```bash
git config --global user.name "Your Name"
git config --global user.email "email@example.com"
```
- 创建版本库并添加文件
```bash
mkdir xxx        # 创建空目录
cd xxx           # 进入目录
pwd              # 显示当前路径
git init         # 将目录变为 Git 仓库
git add xxx      # 添加文件到暂存区
git commit -m "提交说明"   # 提交到本地仓库
```
### 5.4 常用操作（2）
- 三种编译方法：
- g++编译：
```bash
g++ xxx.cpp -o xx    # 源文件编译为可执行文件
./xx                 # 运行可执行文件
# 注：如果不写 -o，默认会生成 a.out
```
- CMake编译：（注意要写CMakeLists.txt）
```bash
mkdir build
cd build
cmake ..
make                
./xx                 # 运行程序
```
- ROS2 colcon 编译
```bash
cd ~/xx              # 进入工作空间根目录
rosdep install --from-paths src --ignore-src -r -y   # 安装依赖（通常只有第一次需要）
colcon build         # 编译
source install/setup.bash                            # 刷新环境
ros2 run xx（包名） xxx（可执行文件名）                       # 运行
```
- git常用操作
- 管理文件
```bash
git status               # 查看仓库状态（显示被修改的文件）
git diff xxx             # 查看文件具体改动内容
git checkout -- xxx      # 撤销工作区修改
git reset HEAD xxx       # 撤销暂存区修改
git rm xxx               # 删除文件
git commit               # 提交删除操作
```
- 远程仓库（建议终端输入code . 在VScode里打开文件，写好.gitignore文件之后，用VScode新建的终端执行操作）
```bash
# 关联远程仓库
git remote add origin xxx远程仓库地址

# 首次推送
git push -u origin xxx分支名

# 删除远程库
git remote -v            # 查看远程库信息
git remote rm xxx        # 删除与本地关联的远程库别名（xxx为库名）
```
- 分支管理
```bash
git checkout -b xxx      # 创建并切换到新分支
git checkout xxx         # 仅切换分支
git branch xxx           # 仅创建分支
git branch               # 查看所有分支
git merge xxx            # 将 xxx 合并到当前分支
git branch -d xxx        # 删除 xxx 分支
```
- 标签管理（其实较少用到）
```bash
git tag v1.0             # 打标签
git tag                  # 查看所有标签
git tag -d v0.1          # 删除标签
git push origin --tags   # 推送所有标签到远程
```
- 推送踩坑：
再次推送的时候，如果被拒绝，就输入git pull --rebase 解决冲突后再次推送

## 6. 宿主机环境配置

### 6.1 深度学习环境配置
- 安装miniconda、cuda和pytorch：<https://blog.csdn.net/clayt/article/details/132326894>
- （conda环境创建的部分我只进行到第九步）
### 6.2 千里队内标注工具
- Gitee: <https://gitee.com/fourzkw/label-creator>
- Github: <https://github.com/fourzkw/LabelCreator>
### 6.3 YOLOv8环境
- 打开Anaconda Prompt（按win键之后可以搜索到）
```bash
# 更新系统包
conda update --all
# 安装Python及相关工具（以3.9为例）
conda install python=3.9
# 创建虚拟环境
conda create -n yolo python=3.9

# 激活虚拟环境
conda activate yolo
#安装依赖项
pip install pyyaml typeguard

# 安装Ultralytics库（包含YOLOv8）
pip install ultralytics

# 安装其他可能需要的库
pip install opencv-python matplotlib
```
### 6.4 常用操作（3）
- 激活环境（每次打开新终端都需要执行）
```bash
conda activate yolo
```
- 进入目录
```bash
cd /d xxx
#（或者cd xxx，/d用来跨盘）
```

## 7. 数据集制作与 YOLO 训练

### 7.1 怎么打标注
- 在终端打开千里队内工具的根目录，输入
```bash
python main.py
```
- 在工具里打开装好图片的文件夹，新建类型之后画框框住你想要识别的物品，处理完所有的图片之后按保存全部，一般在你存放图片的文件夹里会出现data.yaml文件
- 然后，找到生成出来的labels文件夹，在你的dataset_images（取一个别的名字也可以）文件夹里创建好结构

### 7.2 数据集结构

```text
D:/Yolo/dataset_images/    #这里是你的数据集根目录
├── data.yaml
├── images/           #存放图片的文件夹
│   ├── train/        #训练集图片（建议放总图片的八成）
│   └── val/          #验证集图片（建议放总图片的一成）
└── labels/           #存放标注文件（顺序和images文件夹里放的图片必须对应）
    ├── train/        #训练集标签
    └── val/          #验证集标签
```
- 剩下的一成图片建议用来推理验证，总的图片数量不能太少
- 注意data.yaml的内容，尤其要分清谁是0谁是1（）

### 7.3 YOLO训练命令

```bash
yolo task=detect mode=train model=yolov8n.pt data=D:/Yolo/dataset_images/data.yaml epochs=50 imgsz=640 batch=8 workers=0
```
<!-- （其中data=之后填写你的data.yaml文件绝对地址，训练模型会生成在当前目录的一个叫runs的文件夹里） -->
- 说明：
- `model`后面写不写`yolov8n.pt`的绝对地址取决于你当前终端所在的目录
- `data`应该指向data.yaml所在的地址
- `epochs=50`:训练轮数，不宜过大，防止过拟合。
- `batch=8`:批次大小。
- `imgsz=640`:输入图像尺寸，YOLO 会将图片统一缩放至此尺寸。
- `workers=0`:取0值在Windows系统很常见，在别的系统也许可以修改。

### 7.4 推理验证

```bash
yolo task=detect mode=predict model=best.pt source=test.jpg conf=0.25
```
- 说明：
- `model`后面写不写模型的绝对地址取决于你当前终端所在的目录
- `source`之后可以填写文件夹的地址来同时验证多个图片
- `conf`指置信度阈值，低于此值不会画框，可以灵活调整

## 8. 待补章节

<!-- 硬件到货后补：硬件部署、ROS2 跨机通信、PnP 解算、与电控机械联动、性能测试 -->

- [ ] 硬件部署与摄像头读取
- [ ] ROS2 跨机通信测试
- [ ] PnP 解算与小道具坐标发送
- [ ] 与电控/机械联动
- [ ] 性能测试
