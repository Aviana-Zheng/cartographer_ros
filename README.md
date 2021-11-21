
参考：

https://github.com/cartographer-project/cartographer/tree/7f4f78e4513d583b4a1a90f76014db26871cf49e

https://github.com/cartographer-project/cartographer_ros/tree/2441fb585419a7dbda23c44492df02f6287d50b4

https://github.com/slam-code/cartographer

https://github.com/slam-code/SLAM/tree/master/9-cartographer-%E6%BA%90%E7%A0%81%E5%88%86%E6%9E%90


cartographer:

on Jun 23, 2017    

cartographer:

7f4f78e4513d583b4a1a90f76014db26871cf49e

cartographer_ros:

08ff3f9c423fc44855a221638e082219bb3f9e27



参考[官网](https://google-cartographer.readthedocs.io/en/latest/index.html)安装依赖

On Ubuntu 18.04 (Bionic)：

# 1.安装ros(melodic)

# 2.安装依赖

```shell
sudo apt-get update
sudo apt-get install -y \
	build-essential   \
	clang \
	cmake  \
	cmake-gui  \
	freeglut3-dev   \
	g++  \
        git  \
        google-mock \
        libavcodec-dev   \
        libavformat-dev   \
        libboost-all-dev  \
        libcairo2-dev  \
        libceres-dev  \
        libeigen3-dev    \
        libflann1.8(1.9)   \
        libflann-dev     \
        libgflags-dev   \
        libgoogle-glog-dev  \
        libgtest-dev    \
        libgtk2.0-dev    \
        liblua5.2-dev   \  
        libswscale-dev   \
        libusb-1.0-0-dev \
        libusb-dev      \
        libudev-dev   \
	libpcap-dev  \
	libprotobuf-dev   \  
        libqglviewer-dev \ 
	libqt4-dev  \
	libqhull-dev    \
        libqhull7      \
        libqhull-doc     \
        libqhull-r7   \ 
        libsuitesparse-dev  \
        libxmu-dev    \
        libxi-dev    \
        libwebp-dev \
        linux-libc-dev   \
        lsb-release \
        mono-complete   \
        mpi-default-dev      \
        ninja-build  \
        openjdk-8-jdk \
        openjdk-8-jre   \
        openmpi-bin     \
        openmpi-common    \  
        pkg-config   \
        protobuf-compiler \
        python-dev    \
        python-numpy   \
        python-sphinx   \
	python-wstool   \
	python-rosdep    \
	python3-sphinx    \
	qtdeclarative5-dev  \
        qt4-qmake  \
        qt4-dev-tools \
        qt4-doc      \
        qt4-qtconfig    \
        qt4-demos       \
        qt4-designer    \
        qt5-qmake      \
        qtcreator   \
        stow
```

# 3.安装`octomap`:

安装对应ROS版本，比如`octomap_ros-kinetic-devel  octomap_msgs-kinetic-devel  octomap_mapping-kinetic-devel    octomap-devel`

uri: https://github.com/OctoMap/octomap

version: 0ee848bacb2431ae8b4987669ee8fe7a799ad78a

1、解压`Octomap.zip`;

2、打开终端运行命令：

```shell
cd octomap-devel
mkdir build&&cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release ..
sudo make install
```

# 4.安装`ceres-solver-1.14.0`:

1、将`ceres`压缩包解压到指定文件夹下 

2、进入解压的文件夹内，并创建`release`文件夹

```shell
cd ceres  
mkdir release   
```

3、进入`release`目录(之后安装`ceres`的所有文件都会被放到`release`文件夹内)

```shell
cd release  
```

4、`cmake`编译`ceres`源码

```shell
cd release 
cmake .. -DCMAKE_BUILD_TYPE=Release
```

 5、安装

```shell
sudo make -j4 install 
```

# 5.安装gto

uri: https://github.com/RainerKuemmerle/g2o

version: 4b9c2f5b68d14ad479457b18c5a2a0bce1541a90

1、将`g2o`压缩包解压到指定文件夹下 

2、进入解压的文件夹内，并创建`release`文件夹

```shell
cd g2o  
mkdir release  
```

3、进入`release`目录(之后安装`g2o`的所有文件都会被放到`release`文件夹内)

```shell
cd release  
```

4、`cmake`编译`pcl`源码

```shell
cd release 
cmake .. -DCMAKE_BUILD_TYPE=Release 
```

5、安装

```shell
sudo make -j4 install  
```

# 6.安装opencv-2.4.13

uri:  https://github.com/opencv/opencv/tree/2.4.13

version:2.4.13

1、将`opencv-2.4.13.zip`压缩包解压到指定文件夹下 

2、进入解压的文件夹内，并创建`release`文件夹


```shell
cd opencv-2.4.13
mkdir release  
```

3、进入`release`目录(之后安装`OpenCV`的所有文件都会被放到`release`文件夹内)


```shell
cd release  
```

4、`cmake`编译`OpenCV`源码，安装所有的`lib`文件都会被安装到`/usr/local`目录下


```shell
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=OFF -D ENABLE_PRECOMPILED_HEADERS=OFF ..  
```

5、安装


```shell
sudo make install  
```

到此安装完成,以下步骤为测试软件是否可正常运行

6、在某个新建文件夹下建立一个`test.cpp`文件,在终端输入以下命令

```shell
gedit test.cpp
```

在该文档中添加以下内容,保存后关闭文档.

```c++
#include <cv.h>  
#include <highgui.h>  

using namespace cv;  

int main(int argc, char* argv[])  
{  
    Mat image;  
    image = imread(argv[1], 1); 
    if (argc != 2 || !image.data)   
    {  
        printf("No image data\n");  
        return -1;  
    }  
 
    namedWindow("Display Image", CV_WINDOW_AUTOSIZE);  
    imshow("Display Image", image);  
    waitKey(0);  
    return 0;  
}
```

7、在同一目录下再新建一个`cmake`的`CMakeLists.txt`,在终端执行以下命令

```shell
gedit CMakeLists.txt
```

在该文档中输入以下内容,保存后关闭文档

```cmake
cmake_minimum_required(VERSION 2.8)
project(test)  
set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(test test)  
target_link_libraries(test ${OpenCV_LIBS})  
```

8、编译+运行,在终端执行以下两条命令(同一目录下)

```shell
mkdir build 
cd build
cmake ..
make  
```

得到可执行文件`test`(类似于`Windows`系统下的可执行文件`exe`)

9、找一个`jpg`格式的图片做测试，注意要和上面那个`CMakeLists.txt`文件放在同一目录下面,图片名取`test.jpg`。

10、在终端执行以下命令行

```shell
./test   ../test.jpg    
```

如果能看到照片，那就表示成功了。



 # 7.安装pcl-1.8

uri:https://github.com/PointCloudLibrary/pcl/tree/pcl-1.8.0

version:pcl-1.8.0

1、将`pcl1.8`压缩包解压到指定文件夹下 

2、进入解压的文件夹内，并创建`release`文件夹

```shell
cd pcl  
mkdir release 
```

 3、进入`release`目录(之后安装`pcl`的所有文件都会被放到`release`文件夹内)

```shell
cd release  
```

4、`cmake`编译`pcl`源码，安装所有的`lib`文件都会被安装到`/usr/include`目录下

如果提前安装了`cuda`，又不打算在`pcl`中用`cuda`（主要用于`GPU`函数计算提速)，可以用这种方法：

在`CmakeLists.txt`程序关于`cuda`的代码将其注释掉（删除掉）

```cmake
# Cuda
option(WITH_CUDA "Build NVIDIA-CUDA support" TRUE)
if(WITH_CUDA)
  include("${PCL_SOURCE_DIR}/cmake/pcl_find_cuda.cmake")
endif(WITH_CUDA)

option(WITH_QT "Build QT Front-End" TRUE)
if(WITH_QT)
  set(PCL_QT_VERSION 5 CACHE STRING "Which QT version to use")
  if("${PCL_QT_VERSION}" STREQUAL "4")
    find_package(Qt4)
    if (QT4_FOUND)
      include("${QT_USE_FILE}")
    endif (QT4_FOUND)
  elseif("${PCL_QT_VERSION}" STREQUAL "5")
    include(cmake/pcl_find_qt5.cmake)
  else()
    message(SEND_ERROR "PCL_QT_VERSION must be 4 or 5")
  endif()
endif(WITH_QT)
```

编译:

```shell
cd release  
cmake -DCMAKE_BUILD_TYPE=None -DCMAKE_INSTALL_PREFIX=/usr \  
      -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON \  
      -DCMAKE_INSTALL_PREFIX=/usr ..  
```

5、安装

```shell
sudo make -j4 install  
```

6、补充，如果有需要：
打开`PCL\include\pcl-1.7\pcl\impl\point_types.hpp`文件，如下为需要补充的部分代码

```c++
 /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.
 \ingroup common
*/
struct EIGEN_ALIGN16 _PointXYZI
{
PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
union
{
struct
{
  float intensity;
  float azimuth;  //需添加的内容
  float range;    //需添加的内容
  float passibility; //需添加的内容
};
float data_c[4];
};
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
```


对于`pcl1.8`版本及以上的需要找到`point_types.hpp`相应代码的位置，在文件中的对应部分添加。

7、Test
	用文件夹`RGB_D_Slam_test.zip`测试，在`build`里

```shell
cmake ..    
make 
```


 在`bin`中会生成俩个可执行文件`./generate_pointcloud` 与`./main`

# 8.Build and install abseil-cpp 

uri:https://github.com/abseil/abseil-cpp.git

version: d902eb869bcfacc1bad14933ed9af4bed006d481

```shell
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
git checkout d902eb869bcfacc1bad14933ed9af4bed006d481
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local/stow/absl \
  ..
ninja
sudo ninja install
cd /usr/local/stow
sudo stow absl
```



# 9.Build and install proto3

uri:https://github.com/google/protobuf.git

version:v3.4.1

```shell
VERSION="v3.4.1"

# Build and install proto3.
git clone https://github.com/google/protobuf.git
cd protobuf
git checkout tags/${VERSION}
mkdir build
cd build
cmake -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
ninja
sudo ninja install
```



# 10.Build and install Cartographer.

uri: https://github.com/cartographer-project/cartographer.git

version: 7f4f78e4513d583b4a1a90f76014db26871cf49e

```shell
git clone https://github.com/cartographer-project/cartographer.git
cd cartographer
git checkout 7f4f78e4513d583b4a1a90f76014db26871cf49e
mkdir build
cd build
cmake .. -G Ninja
ninja
ninja test
sudo ninja install
```



# 11.新建catkin_ws

```shell
mkdir catkin_ws
cd catkin_ws
wstool init src
```



# 12.Merge the cartographer_ros.rosinstall file 

```shell
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
```

修改上述命令下载的`catkin_ws/src`文件夹下的`.rosinstall`文件如下：

`.rosinstall`原文件是下载最新版本的`cartographer`和`cartographer_ros`。

```c++
# THIS IS AN AUTOGENERATED FILE, LAST GENERATED USING wstool ON 2021-11-19
- git:
    local-name: cartographer
    uri: https://github.com/cartographer-project/cartographer.git
    version: master
- git:
    local-name: cartographer_ros
    uri: https://github.com/cartographer-project/cartographer_ros.git
    version: master
```

`.rosinstall`更换成下面的内容，下载`Jun 23, 2017`号的`cartographer`和`cartographer_ros`，以和[slamcode](https://github.com/slam-code/cartographer)博客保持一致，加快学习的进度。

```shell
# THIS IS AN AUTOGENERATED FILE, LAST GENERATED USING wstool ON 2021-11-19
- git:
    local-name: cartographer
    uri: https://github.com/cartographer-project/cartographer.git
    version: 7f4f78e4513d583b4a1a90f76014db26871cf49e
- git:
    local-name: cartographer_ros
    uri: https://github.com/cartographer-project/cartographer_ros.git
    version: 08ff3f9c423fc44855a221638e082219bb3f9e27
```



# 13.获取源码



```shell
wstool update -t src
```



# 14.其他依赖



```shell
# Install deb dependencies.

# The command 'sudo rosdep init' will print an error if you have already

# executed it since installing ROS. This error can be ignored.

sudo rosdep init
rosdep update
# 下面这一行失败也可以编译成功  --rosdistro=${melodic}
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```



# 15.Build and install.

```shell
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```



# 16.下载数据

https://google-cartographer-ros.readthedocs.io/en/latest/data.html#d-cartographer-backpack-deutsches-museum

如：

`cartographer_paper_revo_lds.bag`         3M 

`taurob_tracker_simulation.bag  `         42.4M

`2011-09-15-08-32-46.bag  `               3.66G

`b3-2016-04-05-14-14-00.bag    `          8.43G

`cartographer_paper_deutsches_museum.bag `      470.5M 



# 17.运行demo

2D:

将`bag`包换成自己的地址

```shell
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=/home/zy/datasets/cartographer/2D/b0-2014-07-21-12-42-53.bag
```

3D:

将`bag`包换成自己的地址

```shell
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=/home/zy/datasets/cartographer/3D/b3-2016-04-05-14-14-00.bag
```



`Cartographer` 现已经支持 `Toyota HSR`、`TurtleBots`、`PR2`、`Revo LDS `这几个机器人平台。

