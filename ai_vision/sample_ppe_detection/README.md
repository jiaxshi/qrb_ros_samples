<div>
  <h1>AI Sample PPE Detection</h1>
  <p align="center">
  </p>
</div>

---

## 👋 Overview

- This sample detects **PPE (Personal Protective Equipment)** — currently **helmet** and **vest** — from an input image or video stream. It bridges an incoming `/image_raw` (`sensor_msgs/Image`) topic to the QRB ROS NN Inference node, runs the model on the Qualcomm **NPU (HTP)** via QNN, and publishes annotated detection results.
- The sample takes an RGB frame from either a local test image (via `image_publisher`) or a video file / live camera (via the built-in `publish_test_video` node), performs letterbox preprocessing, sends the tensor to `qrb_ros_nn_inference`, then post-processes the output tensors (dequantize → confidence filter → per-class NMS) and publishes:
  - `/ppe_detection/image` — the annotated image with bounding boxes.
  - `/ppe_detection/result` — a JSON string describing each detection.
- The model used is a **Gear Guard Net** detection model, provided as a precompiled **QNN context binary** (`gear_guard_net_ctx.bin`).

| Node Name | Function |
| --------- | -------- |
| [image publisher](https://github.com/ros-perception/image_publisher) | Publishes a local image file to a ROS topic at a fixed rate. |
| publish_test_video | Built-in node that reads a video file (or live camera `/dev/videoX`) and publishes frames to `/image_raw` at a low, NPU-friendly rate. |
| sample ppe detection | Subscribes to input images for letterbox preprocessing, sends tensors to the NN inference node, then post-processes (dequantize / NMS / box-hold) and publishes annotated results. |
| [qrb ros nn inference](https://github.com/qualcomm-qrb-ros/qrb_ros_nn_inference) | Loads the trained QNN model, receives preprocessed tensors, performs inference on the NPU, and publishes output tensors. |

## 🔎 Table of contents

- [👋 Overview](#-overview)
- [🔎 Table of contents](#-table-of-contents)
- [⚓ Used ROS Topics](#-used-ros-topics)
- [🎯 Supported targets](#-supported-targets)
- [✨ Installation](#-installation)
- [🚀 Usage](#-usage)
- [👨‍💻 Visualization](#-visualization)
- [👨‍💻 Prerequisites](#-prerequisites)
- [👨‍💻 Build from source](#-build-from-source)
- [🤝 Contributing](#-contributing)
- [❤️ Contributors](#️-contributors)
- [❔ FAQs](#-faqs)
- [📜 License](#-license)

## ⚓ Used ROS Topics

| ROS Topic | Type | Description |
| --------- | ---- | ----------- |
| `/image_raw` | `<sensor_msgs.msg.Image>` | Input image / video frame (subscribed) |
| `qrb_inference_input_tensor` | `<qrb_ros_tensor_list_msgs.msg.TensorList>` | Preprocessed (letterboxed, NHWC uint8) input tensor sent to NN inference |
| `qrb_inference_output_tensor` | `<qrb_ros_tensor_list_msgs.msg.TensorList>` | Raw model output tensors (`boxes`, `scores`, `class_idx`) |
| `/ppe_detection/image` | `<sensor_msgs.msg.Image>` | Annotated image with drawn bounding boxes |
| `/ppe_detection/result` | `<std_msgs.msg.String>` | JSON detection result (`count` + per-box `label` / `score` / `box`) |

## 🎯 Supported targets

<table>
  <tr>
    <th>Development Hardware</th>
    <th>Hardware Overview</th>
  </tr>
  <tr>
    <td>Qualcomm Dragonwing™ IQ-9075 EVK</td>
    <td>
      <a href="https://www.qualcomm.com/products/internet-of-things/industrial-processors/iq9-series/iq-9075">
        <img src="https://s7d1.scene7.com/is/image/dmqualcommprod/dragonwing-IQ-9075-EVK?$QC_Responsive$&fmt=png-alpha" width="160">
      </a>
    </td>
  </tr>
</table>

## ✨ Installation

> [!IMPORTANT]
> The following steps need to be run on **Qualcomm Ubuntu** and **ROS Jazzy**.<br>
> Refer to [Install Ubuntu on Qualcomm IoT Platforms](https://ubuntu.com/download/qualcomm-iot) and [Install ROS Jazzy](https://docs.ros.org/en/jazzy/index.html) to setup environment. <br>
> For Qualcomm Linux, please check out the [Qualcomm Intelligent Robotics Product SDK](https://docs.qualcomm.com/bundle/publicresource/topics/80-70018-265/introduction_1.html?vproduct=1601111740013072&version=1.4&facet=Qualcomm%20Intelligent%20Robotics%20Product%20(QIRP)%20SDK) documents.

- Add qcom ppa repository source:
```bash
sudo add-apt-repository ppa:ubuntu-qcom-iot/qcom-ppa
sudo add-apt-repository ppa:ubuntu-qcom-iot/qirp
sudo apt update
```

- Install the PPE detection Debian package:
```bash
sudo apt install -y ros-jazzy-sample-ppe-detection
```

## 🚀 Usage

> [!IMPORTANT]
> The model must be a **precompiled QNN context binary** (`.bin`), **NOT** the raw `.dlc` file.<br>
> The default model path is `/opt/model/gear_guard_net_ctx.bin`. See [Prerequisites](#-prerequisites) for how to obtain / convert it.

- First check that the three pre-installed dependencies are present in the image:
```bash
ros2 pkg list | grep -E "qrb_ros_nn_inference|qrb_ros_tensor_list_msgs|image_publisher"
image_publisher
qrb_ros_nn_inference
qrb_ros_tensor_list_msgs
```

> [!NOTE]
> `qrb_ros_nn_inference` hard-codes its tensor topics as **absolute** names (`/qrb_inference_input_tensor`, `/qrb_inference_output_tensor`), so they are never affected by a namespace. The detection node uses **relative** tensor names, so run both sides **without** a namespace (no `-r __ns:=...`) and remap the detection node's relative names onto the two absolute paths — otherwise the two sides never match and the inference chain silently breaks.

<details>
  <summary>Run PPE detection on a single image</summary>

**Terminal 1** — start the component container in the background, then load the C++ inference node into it:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rclcpp_components component_container &
```

```bash
ros2 component load /ComponentManager qrb_ros_nn_inference qrb_ros::nn_inference::QrbRosInferenceNode -p backend_option:=/usr/lib/libQnnHtp.so -p model_path:=/opt/model/gear_guard_net_ctx.bin
```

`Inference init successfully!` means the node is ready: it now subscribes `/qrb_inference_input_tensor` and publishes `/qrb_inference_output_tensor`. `component load` is a one-shot command — the resident process is the background `component_container`.

**Terminal 2** — start the detection node (tensor topics remapped to the absolute paths):
```bash
source /opt/ros/jazzy/setup.bash
ros2 run sample_ppe_detection ppe_detection_node --ros-args \
    -r qrb_inference_input_tensor:=/qrb_inference_input_tensor \
    -r qrb_inference_output_tensor:=/qrb_inference_output_tensor
```

**Terminal 3** — publish the test image with the system `image_publisher`:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run image_publisher image_publisher_node --ros-args \
    -p filename:=/data/images/original.jpg \
    -p rate:=1.0 \
    -r image_raw:=/image_raw
```

</details>

<details>
  <summary>Run PPE detection on a video stream</summary>

**Terminal 1** — same as above (container + inference node):
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rclcpp_components component_container &
```

```bash
ros2 component load /ComponentManager qrb_ros_nn_inference qrb_ros::nn_inference::QrbRosInferenceNode -p backend_option:=/usr/lib/libQnnHtp.so -p model_path:=/opt/model/gear_guard_net_ctx.bin
```

**Terminal 2** — detection node, with detection / recording parameters:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run sample_ppe_detection ppe_detection_node --ros-args \
    -r qrb_inference_input_tensor:=/qrb_inference_input_tensor \
    -r qrb_inference_output_tensor:=/qrb_inference_output_tensor \
    -p conf_thresh:=0.5 \
    -p box_hold_frames:=5 \
    -p save_video_path:=/data/videos/ppe_demo_smooth.avi \
    -p output_fps:=10.0
```

**Terminal 3** — publish the video frames:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run sample_ppe_detection publish_test_video --ros-args \
    -p video_path:=/data/videos/original_small.avi \
    -p rate:=10.0 \
    -p loop:=false
```

Pass `-p video_path:=0` (or `/dev/video0`) to use a live camera instead of a file.

</details>

### Node parameters

`ppe_detection_node`:

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `conf_thresh` | `0.5` | Confidence threshold for detections |
| `iou_thresh` | `0.5` | IoU threshold for per-class NMS |
| `box_hold_frames` | `5` | Reuse last detected box for N frames to reduce flicker. `0` = off |
| `save_path` | `""` | If set, overwrite-save the latest annotated frame to this image file |
| `save_video_path` | `""` | If set, record annotated frames into this video file (`.avi` / MJPG recommended) |
| `output_fps` | `2.0` | Frame rate of the recorded demo video (match the publish rate) |

`publish_test_video`:

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `video_path` | `/data/videos/ppe_test.mp4` | Video file path, or `0` / `/dev/video0` for a live camera |
| `rate` | `2.0` | Frames published per second (Hz). Keep low — NPU inference is serial |
| `loop` | `true` | Loop the video when it ends (continuous testing) |
| `frame_step` | `0` | Publish every Nth source frame. `0` = auto from source fps / rate |

Inference node (`qrb_ros_nn_inference`):

| Parameter | Value used here | Description |
| --------- | --------------- | ----------- |
| `backend_option` | `/usr/lib/libQnnHtp.so` | QNN backend library (HTP = NPU) |
| `model_path` | `/opt/model/gear_guard_net_ctx.bin` | Precompiled QNN context binary (`.bin`) |

## 👨‍💻 Visualization

- Because the nodes run without a namespace, the annotated image is on `/ppe_detection/image` and the JSON result on `/ppe_detection/result`. You can view the image in rqt.
Please refer to the [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) to install rqt.

- Alternatively, inspect the result directly from the command line:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /ppe_detection/result
```

- Headless boards (SSH only): use `-p save_path:=/data/latest_frame.jpg` and/or `-p save_video_path:=/data/videos/ppe_demo_smooth.avi` on the detection node and copy the file off the board.

<details>
  <summary>Build from source usage details</summary>

## 👨‍💻 Prerequisites

- Prepare the PPE detection model. The node expects a **precompiled QNN context binary** at `/opt/model/gear_guard_net_ctx.bin`.
  If you only have the raw `.dlc` model, convert it to a context binary:
```bash
sudo mkdir -p /opt/model
qnn-context-binary-generator \
    --backend /usr/lib/libQnnHtp.so \
    --model /usr/lib/libQnnModelDlc.so \
    --dlc_path <your_model.dlc> \
    --binary_file gear_guard_net_ctx \
    --output_dir /opt/model
```

- Add qcom ppa repository source:
```bash
sudo add-apt-repository ppa:ubuntu-qcom-iot/qcom-ppa
sudo add-apt-repository ppa:ubuntu-qcom-iot/qirp
sudo apt update
```

- Install QRB ROS packages and build tools:
```bash
sudo apt install -y ros-jazzy-qrb-ros-nn-inference ros-jazzy-qrb-ros-tensor-list-msgs ros-jazzy-image-publisher
sudo apt install -y ros-dev-tools
sudo rosdep init
rosdep update
```

## 👨‍💻 Build from source

- Download source code from the qrb-ros-sample repository:
```bash
mkdir -p ~/qrb_ros_sample_ws/src && cd ~/qrb_ros_sample_ws/src
git clone -b jazzy-rel https://github.com/qualcomm-qrb-ros/qrb_ros_samples.git
```

- Build the sample from source code:
```bash
cd ~/qrb_ros_sample_ws/src/qrb_ros_samples/ai_vision/sample_ppe_detection

rosdep install --from-paths . --ignore-src --rosdistro jazzy -y --skip-keys "qrb_ros_nn_inference"
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

- Run PPE detection using the three-terminal flow described in [Usage](#-usage) (container + inference node / detection node / image or video publisher), after sourcing the local workspace:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Running without a build (pure runtime image, e.g. Yocto)

`sample_ppe_detection` is a pure Python package, so on a board that has no `colcon` and a read-only `/opt/ros/...` you can skip building entirely and run the source files directly with `python3`. The trade-off is that you must start the three processes manually (no `ros2 launch`) and remap the tensor topics yourself.

**Terminal 1** — container + inference node:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rclcpp_components component_container &
```

```bash
ros2 component load /ComponentManager qrb_ros_nn_inference qrb_ros::nn_inference::QrbRosInferenceNode -p backend_option:=/usr/lib/libQnnHtp.so -p model_path:=/opt/model/gear_guard_net_ctx.bin
```

**Terminal 2** — detection node straight from source:
```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws/src/sample_ppe_detection/sample_ppe_detection
python3 ppe_detection_node.py --ros-args \
    -r qrb_inference_input_tensor:=/qrb_inference_input_tensor \
    -r qrb_inference_output_tensor:=/qrb_inference_output_tensor
```

**Terminal 3** — image input:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run image_publisher image_publisher_node --ros-args \
    -p filename:=/data/images/original.jpg \
    -p rate:=1.0 \
    -r image_raw:=/image_raw
```

or video input:
```bash
source /opt/ros/jazzy/setup.bash
cd /ros2_ws/src/sample_ppe_detection/sample_ppe_detection
python3 publish_test_video.py --ros-args \
    -p video_path:=/data/videos/original_small.avi \
    -p rate:=10.0 \
    -p loop:=false
```

> [!NOTE]
> On a `-sh` shell, backslash line continuation is unreliable — write each command on a single line.

</details>

## 🤝 Contributing

We love community contributions! Get started by reading our [CONTRIBUTING.md](CONTRIBUTING.md).<br>
Feel free to create an issue for bug reports, feature requests, or any discussion 💡.

## ❤️ Contributors

Thanks to all our contributors who have helped make this project better!

<table>
  <tr>
    <td style="text-align: center;">
      <a href="https://github.com/hangshen">
        <img src="https://github.com/hangshen.png" width="100" height="100" alt="Hang Shen"/>
        <br />
        <sub><b>Hang Shen</b></sub>
      </a>
    </td>
  </tr>
</table>

## ❔ FAQs

<details>
<summary>The video "loops forever" or stops immediately — what's wrong?</summary><br>
Some containers/codecs (especially via the GStreamer backend) don't support frame seeking, so looping by rewinding fails. The node auto-detects this and stops with a hint. Re-encode the video to a seekable format, for example:

```bash
ffmpeg -i in.mp4 -an -c:v mjpeg out.avi
```
</details>

<details>
<summary>Why is the publish rate so low (1~3 Hz)?</summary><br>
NPU (HTP) inference is serial — one frame is processed at a time. Publishing frames faster than the NPU can consume them just causes frame drops (the node warns and skips frames while a previous frame is still being processed). Keep the publisher's <code>rate</code> low to match actual inference throughput.
</details>

<details>
<summary>Detections flicker between frames. How can I stabilize them?</summary><br>
The node includes a temporal "box-hold" mechanism: if a class is missed in the current frame, it reuses the most recent detected box for up to <code>box_hold_frames</code> frames. Increase this value to hold boxes longer, or set it to <code>0</code> to disable:

```bash
ros2 run sample_ppe_detection ppe_detection_node --ros-args \
    -r qrb_inference_input_tensor:=/qrb_inference_input_tensor \
    -r qrb_inference_output_tensor:=/qrb_inference_output_tensor \
    -p box_hold_frames:=10
```
</details>

<details>
<summary>I get no detections at all — how do I debug?</summary><br>
Lower the confidence threshold and enable debug logging to inspect raw scores and candidate boxes:

```bash
ros2 run sample_ppe_detection ppe_detection_node --ros-args \
    -r qrb_inference_input_tensor:=/qrb_inference_input_tensor \
    -r qrb_inference_output_tensor:=/qrb_inference_output_tensor \
    -p conf_thresh:=0.3
```

The node logs dequantized score statistics, pre-NMS candidates, and mapped boxes at <code>DEBUG</code> level. Also verify the model at <code>model_path</code> is a valid QNN <code>.bin</code> context binary (not a raw <code>.dlc</code>).
</details>

<details>
<summary><code>ERROR: Model format NOT support!</code> when loading the inference node</summary><br>
<code>nn_inference_node</code> only uses the QNN API that loads a <b>precompiled context binary</b>; on-the-fly <code>.dlc</code> compilation is only available in the <code>qnn-net-run</code> CLI tool. Passing a <code>.dlc</code> path to <code>model_path</code> therefore fails:

```bash
[ERROR] [nn_inference_node]: ERROR: Model format NOT support!
[ERROR] [ppe_container]: Component constructor threw an exception: could not create publisher...
```

Convert the model to a <code>.bin</code> context binary first (see [Prerequisites](#-prerequisites)). A successful load looks like:

```bash
[QRB INFO] Loading model from binary file: /opt/model/gear_guard_net_ctx.bin
[QRB INFO] /usr/lib/libQnnHtp.so initialize successfully
[QRB INFO] Qnn device initialize successfully
[QRB INFO] Initialize Qnn graph from binary file successfully
[INFO] [nn_inference_node]: Inference init successfully!
```

Note that the <code>.bin</code> is tied to a specific backend (HTP version) and must be regenerated for every new model or hardware platform.
</details>

<details>
<summary>The inference node runs but nothing ever comes back — tensor topics don't match</summary><br>
<code>qrb_ros_nn_inference</code> publishes/subscribes <b>absolute</b> topic names (<code>/qrb_inference_input_tensor</code>, <code>/qrb_inference_output_tensor</code>), which are immune to namespaces. If you launch the detection node inside a namespace (<code>-r __ns:=/xxx</code>) its <b>relative</b> tensor names become <code>/xxx/qrb_inference_...</code> and the chain breaks. Run both sides without a namespace and remap the detection node's relative names onto the absolute paths, as shown in [Usage](#-usage). Verify with:

```bash
ros2 topic info /qrb_inference_input_tensor
ros2 topic info /qrb_inference_output_tensor
```

Each should list one publisher and one subscriber.
</details>

## 📜 License

Project is licensed under the [BSD-3-Clause-Clear](https://spdx.org/licenses/BSD-3-Clause-Clear.html) License. See [LICENSE](./LICENSE) for the full license text.
