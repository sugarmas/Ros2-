# ROS 2 图像处理节点代码解析

ROS 里的图像不是数组，是 `sensor_msgs/msg/Image` 消息：带时间戳、坐标系、编码格式、步长的一串字节。要在 OpenCV 里处理它，必须先「翻译」——这个翻译器就是 `cv_bridge`。

这一篇读的是最短的完整例子：52 行，订阅相机图像、画个圆、再发出去。

> 源码：`...\Program_OpenCV_And_ROS2\5.OpenCV_ROS2_Base\opencv_ros2_test.py`
> 依赖安装说明：同目录 `1.OpenCV_Install\1.OpenCV_Install.txt`

## 一、完整代码

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

class ImageConverter(Node):
    def __init__(self):
        super().__init__('opencv_bridge')
        self.bridge = CvBridge()                       # L14 翻译器
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',                 # L17 彩色图话题
            self.callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'cv_bridge_image', 10)
        self.get_logger().info('cv_bridge_test node started')

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')   # L25
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        (rows, cols, channels) = cv_image.shape          # L30
        if cols > 40 and rows > 40:
            cv2.circle(cv_image, (40, 40), 40, (0, 0, 255), 2)   # 画红圈

        try:
            self.publisher_.publish(
                self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))  # L35 发回去
        except Exception as e:
            self.get_logger().error(f"Failed to convert image back: {e}")

        cv2.circle(cv_image, (40, 40), 20, (0, 255, 0), 2)   # L39 绿圈
        cv2.namedWindow("cv_image", cv2.WINDOW_NORMAL)
        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()
```

## 二、`cv_bridge` 到底做了什么

ROS 图像消息的字段：

```
header         时间戳 + 坐标系
height, width
encoding       'rgb8' / 'bgr8' / 'mono8' / '32FC1' ...
is_bigendian
step           一行多少字节
data           uint8[] 原始字节
```

OpenCV 的 `cv::Mat` / numpy 数组只认「宽高通道 + 数据类型 + 内存布局」。`cv_bridge` 的工作是：

1. 按 `encoding` 决定 numpy 的 dtype 和通道数；
2. 按 `step` 处理行对齐（有时每行末尾有 padding）；
3. 必要时做颜色通道重排。

### 2.1 `bgr8` 与 `rgb8` 的坑

OpenCV 的默认颜色顺序是 **BGR**，而相机驱动（尤其是 RealSense、Astra）和大多数 ROS 工具默认 **RGB**。

```python
cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
```

这句会**自动转换**：如果原消息是 `rgb8`，cv_bridge 把 R 和 B 交换后给你 `bgr8`。所以 OpenCV 里 `(255,0,0)` 是蓝色，与直觉相反。

如果写成 `desired_encoding='passthrough'`，则不转换，原样给——**此时颜色顺序取决于消息本身**，用错会出现「红变蓝」。视觉巡线和跟随的代码里都显式写 `bgr8` 或 `rgb8`，就是在管理这件事。

### 2.2 深度图的编码

深度图常见 `32FC1`（每像素 float32，单位米）或 `16UC1`（uint16，单位毫米）。cv_bridge 转成 numpy 后：

- `32FC1` → `float32`，无效像素是 `NaN`；
- `16UC1` → `uint16`，无效像素常是 `0`。

后面的视觉跟随代码用 `depthFrame[~np.isnan(depthObject)]` 过滤，正是假设了 `32FC1`。

## 三、这次发布与上次画圈：一个容易忽略的顺序

```python
cv2.circle(cv_image, (40, 40), 40, (0, 0, 255), 2)     # 红圈
self.publisher_.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
cv2.circle(cv_image, (40, 40), 20, (0, 255, 0), 2)     # 绿圈
cv2.imshow("cv_image", cv_image)
```

发布出去的画面只有**红圈**，本地窗口里是**红圈 + 绿圈**。因为 `publish` 在第二次画圈之前执行，消息已经序列化完了。

这不是 bug，但值得注意：**`publish` 传的是当时的引用还是拷贝？** 对于 `imgmsg_to_cv2` 得到的 numpy 数组，`cv2_to_imgmsg` 会把它拷进消息对象，所以之后再改 `cv_image` 不影响已发布的消息。如果反过来先画再发，就会把调试用的图形也发给下游——有时这正是你要的（比如给 RViz 看检测结果），有时不是。

## 四、回调里做显示：方便，但会掉帧

`cv2.imshow` + `cv2.waitKey(10)` 在回调里同步执行。`rclpy.spin` 是单线程的，回调没返回就不会处理下一条消息。

假设相机 30 Hz，每帧回调耗时 40 ms（转换 + 画图 + 显示），那么实际处理频率只有 25 Hz，**消息在队列里堆积，延迟越来越大**。队列深度 10 只是缓冲，不是解决。

三种改法：

1. **只在调试时显示**：用参数控制，上线时关掉。
2. **降低分辨率/帧率**：相机端配置。
3. **用多线程执行器**：`MultiThreadedExecutor` + 回调组（callback group），显示和计算分到不同线程。

第 3 种是正解，但要注意共享数据的加锁。

## 五、QoS：最常见的「收不到图像」

```python
self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
```

第四参数 `10` 是简写，等价于 **reliable（可靠）+ depth 10**。而很多相机驱动用的是 **best_effort（尽力而为）**，这是传感器数据的默认 QoS（`qos_profile_sensor_data`）。

**reliable 订阅端 + best_effort 发布端 = 不兼容，收不到任何消息**，而且默认不报错。

排查方法：

```bash
ros2 topic info /camera/color/image_raw -v
```

输出里会列出每个 endpoint 的 QoS。两边 `Reliability` 不一致时，订阅端改成：

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, '/camera/color/image_raw',
                         self.callback, qos_profile_sensor_data)
```

这个问题在「用 `ros2 topic echo` 能看到图像、自己写的节点收不到」时几乎是唯一原因。

## 六、运行与验证

```bash
# 先启动相机
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py

ros2 topic list | grep image
ros2 topic hz /camera/color/image_raw

# 跑本节点
ros2 run <包名> opencv_ros2_test

# 另开终端看发布结果
ros2 run rqt_image_view rqt_image_view
# 选择 /cv_bridge_image
```

如果机器人主控没有显示器，`cv2.imshow` 会报 `cannot open display`。无界面环境下应删掉显示部分，改用 `rqt_image_view` 远程看——这也是「不要在 ROS 节点里强依赖 GUI」的现实原因。

## 七、从这 52 行能带走的模式

- **订阅 → 转换 → 处理 → 转回 → 发布**，是 ROS 视觉节点的标准流水线。
- **`cv_bridge` 是边界**：ROS 用消息，OpenCV 用数组，转换只发生在边界，中间可以随便用 numpy。
- **编码要显式声明**：`bgr8` / `rgb8` / `passthrough` 的选择决定了颜色对不对、深度单位对不对。
- **QoS 必须两边匹配**：传感器数据默认 best effort，自己写订阅时别忘了。
- **回调要短**：显示、保存、网络请求都不该放在高频回调里。

## 相关

- [[ROS 2 视觉巡线代码解析]] —— 在这条流水线后面接了颜色分割和速度控制。
- [[ROS 2 视觉跟随代码解析]] —— 同时处理彩色图与深度图，还要做时间同步。
- [[ROS 2 话题通信代码解析]] —— 订阅/发布的写法基础。
- [[ROS 2 通信机制]] —— QoS 的概念层解释。
- [[ROS Index]]
