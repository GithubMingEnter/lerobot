<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="media/lerobot-logo-thumbnail.png">
    <source media="(prefers-color-scheme: light)" srcset="media/lerobot-logo-thumbnail.png">
    <img alt="LeRobot, Hugging Face Robotics Library" src="media/lerobot-logo-thumbnail.png" style="max-width: 100%;">
  </picture>
  <br/>
  <br/>
</p>

<div align="center">

[![Tests](https://github.com/huggingface/lerobot/actions/workflows/nightly-tests.yml/badge.svg?branch=main)](https://github.com/huggingface/lerobot/actions/workflows/nightly-tests.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/huggingface/lerobot/branch/main/graph/badge.svg?token=TODO)](https://codecov.io/gh/huggingface/lerobot)
[![Python versions](https://img.shields.io/pypi/pyversions/lerobot)](https://www.python.org/downloads/)

[![Status](https://img.shields.io/pypi/status/lerobot)](https://pypi.org/project/lerobot/)
[![Version](https://img.shields.io/pypi/v/lerobot)](https://pypi.org/project/lerobot/)
[![Examples](https://img.shields.io/badge/Examples-green.svg)](https://github.com/huggingface/lerobot/tree/main/examples)
[![Contributor Covenant](https://img.shields.io/badge/Contributor%20Covenant-v2.1%20adopted-ff69b4.svg)](https://github.com/huggingface/lerobot/blob/main/CODE_OF_CONDUCT.md)


</div>



<br/>

<h3 align="center">
    <p>LeRobot: State-of-the-art AI for real-world robotics</p>
</h3>

---



# Daimon IL for Realmen Robot

## Install
```bash
conda create -y -n lerobot_dm python=3.10 && conda activate lerobot_dm
pip install -e ".[feetech]"
pip install rospkg
conda install -y -c conda-forge ffmpeg
pip uninstall -y opencv-python
conda install -y -c conda-forge "opencv>=4.10.0"
```



## Perpare
机器人相关配置文件位于`lerobot/configs/robot/dm.yaml`
* 查找相机口
[参考这个](https://github.com/huggingface/lerobot/blob/main/examples/7_get_started_with_real_robot.md#c-add-your-cameras-with-opencvcamera)
```bash
python lerobot/common/robot_devices/cameras/opencv.py \
    --images-dir outputs/images_from_opencv_cameras
```
查看输出图片编号,会有对应的标记

如相机口为`/dev/video0`，则填入`0`
```yaml
  laptop:
    _target_: lerobot.common.robot_devices.cameras.opencv.OpenCVCamera
    camera_index: 0 # note config index

```

```bash
python lerobot/common/robot_devices/cameras/intelrealsense.py \
    --images-dir outputs/images_from_realsense_cameras
```




* 电机接口查找
```bash
python lerobot/scripts/find_motors_bus_port.py 
```
填入到`lerobot/configs/robot/dm.yaml`中相应的位置

如leader臂夹爪电机口为`/dev/ttyUSB0`，则填入`/dev/ttyUSB0`

注意添加对 USB 端口的访问权限

``` yaml
leader_arms:
  main:
    _target_: lerobot.common.robot_devices.motors.realmen.RealmenMotorBus
    port: /dev/ttyUSB0 # note config port
...

```

* 设置夹爪
注意夹爪的ID为8与配置文件一致
```bash
python lerobot/scripts/configure_motor.py \
  --port /dev/ttyUSB1 \
  --brand feetech \
  --model sts3215 \
  --baudrate 1000000 \
  --ID 8

```
另一个夹爪同样如此

设置舵机的ID为1,也可以设置为8

```Python
python lerobot/scripts/configure_motor.py \
  --port /dev/ttyUSB0 \
  --brand feetech \
  --model sts3215 \
  --baudrate 1000000 \
  --ID 8
```

夹爪电机需要进行校准最大开合角度和完全闭合角度,写入到配置文件中[TODO]
```python
self.open_gripper_angle =  0.0
self.close_gripper_angle = -3.14
```

* 设置环境变量
```bash
export LEROBOT_HOME= xx
export DATA_DIR="$LEROBOT_HOME/data"
export HYDRA_FULL_ERROR=1
export HF_USER=xx
```
写入到.bashrc中,或者在运行时设置.



## Teleoperate

使用下面功能包
```yaml
daimon_dual_arm_bringup 
daimon_dual_arm_moveit_config 
daimon_realman_urdfs 
rm_robot

```

注意rm_robot使用的版本分支为`feature/gripper` 

daimon_realman_urdfs branch:`feature/add_realman_robot_arm`

启动机械臂
```bash
roslaunch daimon_dual_arm_bringup dual_arm_control.launch 
roslaunch daimon_dual_arm_bringup dual_arm_bringup.launch 
```

在realmen类中注意设置 `is_sim = False`


机械臂遥操作:
no camera:
```bash
python lerobot/scripts/control_robot.py teleoperate \
    --robot-path lerobot/configs/robot/dm.yaml \
    --robot-overrides '~cameras' \
    --display-cameras 0

```

```bash
python lerobot/scripts/control_robot.py teleoperate \
    --robot-path lerobot/configs/robot/dm.yaml \

```


## Record
创建data文件夹,
不上传数据 `--push-to-hub 0` ,后面数据文件名为`act_dm_test`
```bash
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/dm.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/act_dm_test \
  --tags dm tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 50 \
  --reset-time-s 10 \
  --num-episodes 5 \
  --push-to-hub 0
```

## Visualize

```bash
python lerobot/scripts/visualize_dataset_html.py --root data --repo-id Swingming/act_dm_test --episode 0
```
![alt text](./media/dm/image.png)


## Train
使用wandb 需要先进行注册登录.
如果不使用wandb
wandb.enable=false 
```bash
python lerobot/scripts/train.py \
  dataset_repo_id=${HF_USER}/act_dm_test \
  policy=act_dm_real \
  env=dm_real \
  hydra.run.dir=outputs/train/act_dm_test \
  hydra.job.name=act_dm_test \
  device=cuda \
  wandb.enable=false
```





## Evaluate

推理时注意设置`-p`,特别是在不同电脑中训练, 会自动记录训练数据
现在的训练数据文件夹名为eval_{use_data_name}

```zsh
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/dm.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/eval_act_dm_test \
  --tags dm tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 50 \
  --reset-time-s 10 \
  --num-episodes 5 \
  -p outputs/train/act_dm_test/checkpoints/last/pretrained_model\
  --push-to-hub 0
```




# Other

## 机器人camera

添加规则：
根据摄像头的唯一标识符，绑定到固定路径。例如：

如对于video0
```bash
udevadm info --name=/dev/video0 --attribute-walk | grep -E "idVendor|idProduct|serial"
```
注意对比，确保ID的唯一性

```
sudo gedit /etc/udev/rules.d/99-camera.rules
```


```json
KERNEL=="video[0,2,4,6,8]*",KERNELS=="7-2", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="3035", MODE:="0777", SYMLINK+="camLeft"
KERNEL=="video[1,3,5,7,9]*",KERNELS=="1-10.1", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="3035", MODE:="0777", SYMLINK+="camRight"
KERNEL=="video[1,3,5,7,9,11]*",KERNELS=="6-2", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0777", SYMLINK+="camWrist"
KERNEL=="video[1,3,5,7,9,11,13]*",KERNELS=="9-2", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5535", MODE:="0777", SYMLINK+="camFishWrist"
KERNEL=="video[1,3,5,7,9,11,13,15]*",KERNELS=="1-10.2", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="5535", MODE:="0777", SYMLINK+="camArm2FishWrist"

```


sudo udevadm control --reload-rules
sudo udevadm trigger


```bash
 ls -l /dev | grep video
lrwxrwxrwx   1 root root             7 1月  13 15:11 camArm2FishWrist -> video16
lrwxrwxrwx   1 root root             7 1月  13 15:11 camFishWrist -> video13
lrwxrwxrwx   1 root root             6 1月  13 15:09 camLeft -> video0
lrwxrwxrwx   1 root root             6 1月  13 15:09 camRight -> video3
```
```
lrwxrwxrwx   1 root root             7 1月  13 15:09 ttyArm2g -> ttyACM3
lrwxrwxrwx   1 root root             7 1月  13 15:09 ttyFollower -> ttyACM1
lrwxrwxrwx   1 root root             7 1月  13 15:09 ttyLeader -> ttyACM2
```
使用 OpenCV 访问绑定的摄像头op0
```bash
import cv2

# 使用固定的设备路径
cap = cv2.VideoCapture('/dev/my_camera')

if not cap.isOpened():
    print("无法打开摄像头")
else:
    print("摄像头已打开")
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Camera", frame)
        cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()
```


## 机器人 USB Serial

---

### **Option 1: Use Symbolic Links**
You can create symbolic links to combine or alias the serial devices. For example, you can create a symbolic link `/dev/ttyCombined` that points to one of the devices.

1. **Create a Symbolic Link**:
   ```bash
   sudo ln -s /dev/ttyACM0 /dev/ttyCombined
   ```

2. **Verify the Link**:
   ```bash
   ls -l /dev/ttyCombined
   ```

   Output:
   ```
   lrwxrwxrwx 1 root root 10 Sep  1 12:00 /dev/ttyCombined -> /dev/ttyACM0
   ```

3. **Use the Combined Link**:
   You can now use `/dev/ttyCombined` in your application to access `/dev/ttyACM0`.

---

### **Option 2: Use Udev Rules to Create Persistent Links**
Udev rules allow you to create persistent symbolic links based on device attributes (e.g., serial number, vendor ID, etc.). This ensures that the links are consistent across reboots.

1. **Identify Device Attributes**:
   From your output, note the `ID_SERIAL_SHORT` and `ID_PATH` attributes for each device:
   - `/dev/ttyACM0`:
     ```
     ID_SERIAL_SHORT=58FA095194
     ID_PATH=pci-0000:00:14.0-usb-0:5:1.0
     ```
   - `/dev/ttyACM1`:
     ```
     ID_SERIAL_SHORT=5959112553
     ID_PATH=pci-0000:00:14.0-usb-0:8:1.0
     ```

2. **Create a Udev Rule**:
   Create a new udev rule file in `/etc/udev/rules.d/`:
   ```bash
   sudo nano /etc/udev/rules.d/99-combined-serial.rules
   ```

   Add the following rules to create symbolic links based on the `ID_SERIAL_SHORT`:
   ```bash
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="58FA095194", SYMLINK+="ttyCombined0"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5959112553", SYMLINK+="ttyCombined1"
   ```

   - This will create `/dev/ttyCombined0` and `/dev/ttyCombined1` for the respective devices.

3. **Reload Udev Rules**:
   Reload the udev rules and trigger them:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

4. **Verify the Links**:
   Check if the symbolic links have been created:
   ```bash
   ls -l /dev/ttyCombined*
   ```

   Output:
   ```
   lrwxrwxrwx 1 root root 10 Sep  1 12:00 /dev/ttyCombined0 -> /dev/ttyACM0
   lrwxrwxrwx 1 root root 10 Sep  1 12:00 /dev/ttyCombined1 -> /dev/ttyACM1
   ```

5. **Use the Combined Links**:
   You can now use `/dev/ttyCombined0` and `/dev/ttyCombined1` in your application.

---

### **Option 3: Combine Devices in Software**
If you need to combine the data streams from multiple serial devices, you can write a script or use a tool like `socat` to merge the data.

1. **Install `socat`**:
   ```bash
   sudo apt install socat
   ```

2. **Combine Serial Streams**:
   Use `socat` to combine the data from `/dev/ttyACM0` and `/dev/ttyACM1`:
   ```bash
   socat /dev/ttyACM0 /dev/ttyACM1
   ```

   This will forward data between the two devices. You can redirect the output to a file or another program if needed.

---

### **Option 4: Use a Custom Script**
If you need more control, you can write a Python script to read from both devices and combine the data.

1. **Install `pyserial`**:
   ```bash
   pip install pyserial
   ```

2. **Python Script**:
   ```python
   import serial

   # Open serial ports
   ser0 = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=1)
   ser1 = serial.Serial('/dev/ttyACM1', baudrate=9600, timeout=1)

   while True:
       # Read from both devices
       data0 = ser0.readline()
       data1 = ser1.readline()

       # Process or combine the data
       if data0:
           print(f"ttyACM0: {data0.decode().strip()}")
       if data1:
           print(f"ttyACM1: {data1.decode().strip()}")
   ```

   - This script reads data from both devices and prints it to the console.

---

