# L2 感知层接入约定

P1 LeverDetector、P2 PipeDetector、P3 SceneSegmenter、P4 LidarProcessor
各为独立常驻线程类。行为层只调用 enable(**算法参数)、read()、disable()；
程序退出时调用 close()。read() 读取最近一次缓存，不触发相机采集。

帧源由外部实现方代码注入，需提供非阻塞的 read() -> (ret, payload, timestamp)，
timestamp 为 Unix 秒，超过 1 秒的旧帧
按 E_SENSOR_NO_DATA 处理。处理器是 (payload, params) -> dict 回调。
P1/P2 的处理器返回相机系三维点；调用方必须注入标定过的 to_dog，
P2 的方向向量另用不含平移的 vector_to_dog。没有外参时返回
E_TF_UNAVAILABLE，不能把相机坐标冒充狗坐标。
P3/P4 的注入处理器须在 L2 内完成各自传感器外参/几何处理，
返回可供 L3 直接使用的狗系相对量及安全状态。

P2 check_detach() 根据最新帧的两根风管端点执行“点数 + 点距”判定：
最新有效帧恰有两个狗系端点且距离严格大于阈值才成功。端点缺失时返回失败与
E_TARGET_NOT_FOUND。没有真实传感器或处理器时，四类只报告错误/未知状态；
当前工程不含 YOLO 分割、OCR、点云聚类、相机外参或外部实现方设备适配。

L3 沿用 BaseAction、八类基础动作及 RunContext，高位/低位排风另有
独立任务类；create_action()
只负责构造单个动作实例，任务顺序、重试和跳过仍由 L4 总控负责。
高低排风任务类在执行机构缺失时直接失败；旧 VentAction 的 dry-run
仅保留供历史 Mock 测试使用，不在新构造入口注册。
所有感知类不得发送底盘或上装运动命令。
