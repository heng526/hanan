# 相机采集源使用说明

`camera_source.py` 是参考提供方 `vCam` 三元组接口的可配置实现。构造后启动一个采集线程；
`read()` 返回 `(ret, frame_copy, Unix_timestamp)`。P1/P2/P3 可以把同一个相机源
作为 `source` 注入，各任务只在 `enable()` 后执行自己的处理器；相机采集线程由调用方
统一管理。P4 雷达仍需单独注入雷达帧源和处理器，本模块没有接入雷达协议。

外部实现方使用时应从现有配置读取参数，不能把下例变量当作真实设备参数：

```python
from perception import CameraSource, LeverDetector

camera = CameraSource(
    src=camera_config["src"],
    image_height=camera_config["height"],
    image_width=camera_config["width"],
    source_kind=camera_config["kind"],  # "device" / "file" / "rtsp"
)
lever = LeverDetector(source=camera, processor=existing_lever_processor,
                      to_dog=calibrated_camera_to_dog)
try:
    lever.enable()
    result = lever.read()  # 非阻塞，需检查 error_code、frame_count、age_s
finally:
    lever.disable()
    lever.close()
    stopped = camera.stop(timeout=2.0)
    if not stopped:
        # 后端 read() 仍阻塞；等待其退出，不要重开同一个源或从别的线程释放 capture。
        pass
```

`source_kind="file"` 的文件到 EOF 后进入 `EOF`，默认不重开；只有显式设置
`loop_file=True` 才会重播。`rtsp` 必须使用 `rtsp://` 或 `rtsps://` 地址，
`device` 使用外部实现方确认的设备标识；这两类读帧或打开失败后按
`reconnect_interval` 重连。`status()` 提供状态、累计帧数、最后有效帧时间、
帧龄、是否新鲜和错误码，不回显可能含凭据的地址。`read()` 对断流和超过
`max_age_s` 的帧返回 `False, None, 最后有效时间`，下游 L2 不得复用旧画面。
文件是否存在、相机是否可打开及 RTSP 凭据均须在外部实现方环境核实。

与参考提供方示例相比，此实现取消黑色错误占位帧，避免被算法当作真图；区分文件 EOF
与实时流失联；限制旧帧；线程安全复制快照；`stop()` 等待采集线程统一释放资源，
重复调用不重复释放。采集后端若永久阻塞在 `read()` 或打开调用，Python 线程
不能安全强杀，`stop(timeout)` 会返回 `False`；需在外部实现方为实际后端配置并验证
超时机制。停止后的源不可重启，应新建实例。

离线验证命令：在 `d50ws_nav` 目录运行
`python -B -m pytest -p no:cacheprovider -q tests/test_camera_source.py`。
测试只注入假采集器，不打开本机相机、文件或网络流。

## 329 外部实现方需要补齐的资料

1. Ubuntu 当前可控狗代码的版本，以及相机、雷达、机械臂和 RunContext 的实际接口。
2. 相机源类型、地址或设备标识、分辨率、帧率、采集后端及打开/读帧超时配置；
   RTSP 凭据的安全配置方式。不要在代码或证据中写入凭据。
3. 相机内参、深度对齐方式、相机到狗坐标系外参与时间同步；P1/P2 的
   `to_dog`/`vector_to_dog` 不能用占位值上机。
4. 已有扳手/管件检测、场景分割、OCR、点云处理器的可调用接口和输出字段，
   以及雷达数据协议、坐标约定和有效期。
5. 在外部实现方独立验收采集、断流、重连和帧龄，再在单独授权及现场安全条件下
   验证任何真机动作。本次工作未执行这些步骤。
