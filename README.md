# 车载平衡滚球控制

MaixCAM2 / UDP / ESP32-S3 / ESP-IDF

[现场演示与项目介绍](https://dgwfpga.cn/showcase/ball-balance.html) · [地瓜丸的 FPGA 教学平台](https://dgwfpga.cn/) · [验证范围](VALIDATION.md)

![项目实景](https://dgwfpga.cn/portfolio-media/20261109/assets/project-field/ball-system.webp)

相机给出球与目标的位置，ESP32-S3 执行周期控制，再由电机改变杆面倾角。这里发布原归档中时间较新的“决定版”相机／控制器配对，不混用早期 C 固件和后期相机协议。

```mermaid
flowchart LR
 V[MaixCAM2 检测 / 目标] -->|UDP 5005| C[ESP32-S3 控制]
 C -->|UART| M[Emm42_V5 电机]
 M --> R[杆面倾角 / 球位置]
 R --> V
```

## 目录和文件对应什么

| 位置 | 对应部分 | 看什么 |
| --- | --- | --- |
| [maixcam/](maixcam/README.md) | 视觉端 | 识别球、设定目标，通过 UDP 发送坐标 |
| [esp32/](esp32/README.md) | 控制与执行端 | 球速估计、控制状态机、倾角与电机通信 |
| [docs/](docs/README.md) | 配置与调试笔记 | 网络配置、参数迭代与控制方法 |
| [README.md](README.md) | 项目总入口 | 先看模块对应关系，再选运行路线 |
| [VALIDATION.md](VALIDATION.md) | 检查记录 | 哪些做过语法／编译检查，哪些还需上板 |
| [NOTICE.md](NOTICE.md) | 来源与依赖 | 平台 SDK、运行库及原有许可 |
| [LICENSE](LICENSE) | 开源许可 | MIT 授权条款与版权署名 |
| [.gitignore](.gitignore) | 本地文件排除规则 | 个人配置、模型、日志和编译生成物 |


## 配置与构建

使用 ESP-IDF 的 ESP32-S3 开发环境。在仓库根目录执行：

```powershell
Copy-Item esp32/main/config.example.h esp32/main/config.h
# 编辑 config.h 中的 WIFI_SSID / WIFI_PASS，再核对 GPIO 与控制方向
cd esp32
idf.py set-target esp32s3
idf.py build
idf.py -p <你的串口> flash monitor
```

`config.h` 被 Git 忽略，不要提交自己的热点凭据。默认值不能直接连接网络。

相机与 ESP32 需在同一网络。相机脚本发送 `B,seq,ball_cm,target_cm` 或 `N,seq`。模型权重不随代码分发：从你自己的训练导出 MaixCAM2 对应模型，核对 `MODEL_FILENAME`、类别标签与部署路径。MaixCAM Pro 与 MaixCAM2 的模型产物不能凭扩展名相似就直接混用。

先核对供电、原点和输出方向，再按 `esp32/README.md` 联调。控制笔记中的目标时间、参数和早期现象不作为本次复测的性能承诺。


## 许可与贡献

项目代码按 [MIT](LICENSE) 发布，依赖与原始署名见 [NOTICE.md](NOTICE.md)。欢迎附上所用硬件、固件／工具版本、复现步骤和日志提出问题；请先去除自己的网络密码和设备信息。
