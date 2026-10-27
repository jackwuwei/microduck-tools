# Microduck 项目约定

## 真机供电与标定

用户已明确确认此机器人沿用 Microduck 官方的 2S 电池直供舵机设计。官方设计范围为 6.6–8.2 V；本机实测约 7.9–8.0 V 属于该设计的正常情况。不要仅因超过 ROBOTIS XL330 的额定 6 V 或出现单独的输入电压报警位 0x01，就拒绝继续、要求降到 6 V、或再次让用户解释/确认。这是项目专用约定，不推广到其他机器人。

依据：https://github.com/pollen-robotics/microduck/blob/main/docs/design/robotd-design.md#32-state-out

标定工作台真机启动使用 `--microduck-2s` 配置，保留电压和错误状态显示。不改写舵机 Shutdown、电压限值，不启用扭矩，不发送运动目标。继续检查其他硬件错误、温度、总线独占，以及偏离项目供电范围的情况。

标定工作台始终运行在电脑上，网页、标定逻辑和备份均留在电脑。默认通过 SSH 别名 `microduck` 访问主控串口 `/dev/ttyS2`，波特率 1000000；主控仅在连接期间通过 SSH 标准输入输出运行内存中的临时串口助手，不部署或启动网页工作台，不使用网页端口转发。远端 Python 解释器及依赖目录通过 `--remote-python`、`--remote-pythonpath` 配置；默认搜索主控账户下的 `~/.local/share/microduck-serial/deps`，并兼容 `~/diagnostics/readlibs`。其次支持电脑通过 USB 舵机串口适配器直连；使用电脑实际串口路径。连接前检查串口拥有者及其他总线程序。不在文件中保存密码。
