# RT1064 智能车 · 算法 16

本仓库包含比赛用车的固件、BOOT2 下载工具和基于车载灰度输入的双向闭环回放。当前相机安装高度 31.5 cm，光轴相对水平向下 30°。

## 从克隆到构建

Windows 下使用 PowerShell 7、Arm GNU Toolchain 15.2、CMake 和 Ninja。将命令加入 PATH 后，在仓库根目录运行：

```powershell
git clone https://github.com/jpage4986-collab/rt1064-smartcar-team.git
cd rt1064-smartcar-team
./build.ps1
```

应用位于 `Project/GCC/wireless-app/tutorial.elf`，引导程序位于 `Project/GCC/wireless-loader/tutorial.elf`。

## 回放与烧录

闭环回放需要 Python、Node.js 和下列 Python 依赖。默认运行顺、逆两个方向，输出每帧位置、状态、舵角和电机占空比，以及每个方向的汇总。仅仿真器和评分器读取赛道图，固件逐帧接收 188 × 120 灰度图。

```powershell
python -m pip install -r sim/requirements.txt
python sim/replay.py --direction both --output reports/replay
```

主板供电、CMSIS-DAP 接好后，在 Windows x64 上使用 Python 3.13 和 PowerShell 7。第一次先运行安装脚本，它在当前用户目录创建独立环境并安装烧录依赖。固件启动后可能立即输出电机 PWM，台架烧录时请架空驱动轮。

```powershell
./tools/RT1064Tools/install.ps1
./flash.ps1
```

[离线安装包（Windows x64、Python 3.13）](https://github.com/jpage4986-collab/rt1064-smartcar-team/releases/download/rt1064tools-0.45.1/RT1064Tools-win64-py313-offline.zip) · [下载器接线和供电说明](tools/RT1064Tools/README.md)

下载脚本先备份将覆盖的扇区，然后用 HEX 写入 BOOT2 布局，校验引导区、应用区、备份应用区和清单，最后读取算法编号、心跳和摄像头帧计数。`reports/flash/latest-wired-update.json` 保存本机烧录凭据。使用自定义 RT1064 下载脚本路径时可传入 `-FlashScript`。

## 队友分工

| 修改范围 | 入口 |
| --- | --- |
| 普通道路边线、目标点和调度 | `Project/USER/src/track_mode16.c` |
| 岛缘识别、进环、绕行和合并 | `Project/USER/src/a16_roundabout.c` |
| 贴边障碍 | `Project/USER/src/a16_obstacle.c` |
| 起跑线、终点和停车锁存 | `Project/USER/src/a16_finish.c` |
| 舵机、电机和持续弯响应 | `Project/USER/src/main.c` |
| 车辆参数 | `Project/USER/inc/tune_config.h` |
| 原赛道双向回放 | `sim/replay.py`、`sim/home-track.png` |

从 main 建分支，提交改动时附双向回放结果。用标签 `vehicle-5ad9d3f3` 查看上一版车载固件对应源码，用 `a16-before-module-split` 对照功能修改与文件拆分，用 `a16-b6072527` 找回 23 cm 机位的烧录版本。

## 31.5 cm、俯视 30° 机位

算法 16 的环岛拟合、入口像素门限和近场循迹共用一个 23 cm、俯视 15° 的参考视图。`a16_camera.c` 根据实际安装姿态将可见地面采样到该坐标系，LCD 的边线和目标也来自同一视图。参数在 `tune_config.h`。模型沿用约 75° 的水平视场角，实车镜头内参和畸变尚未标定。

在确认的赛道图上，旧固件以此新机位从两个方向经过右岛时都保持 ROAD。新固件两个方向均经历 ENTRY、ARC、MERGE 并回到 ROAD，顺时针 77.5 秒、逆时针 75.9 秒终点停稳。以模拟车实际位置计，右岛附近累计绕行分别为 475.2° 和 470.5°。起步向右偏 3 cm、车头顺时针偏 4° 的两次重复测试也均通过。

采用新机位、丢线时立即撤去驱动的前一版 CRC32 为 `4A45DD67`，Git 标签为 `a16-4a45dd67`。

有可靠路线并已驱动后，若后续摄像头画面丢失路线，控制器保留最后的舵机指令，以 17% 基础 PWM 向前找路。连续丢线 2 秒后两路 PWM 清零。重获可信路径后恢复循迹。当前找路版标签为 `a16-c89f00e4`。上电一直无路保持停车，摄像头断帧 250 ms、STOP 和终点停车优先。可用 `python sim/test_bend_loss.py` 复现左右弯道遮挡和重获，用 `python sim/replay.py --direction both` 运行全圈。均为仿真验收，实车通过率仍待测。

## 23 cm、俯视 15° 历史版本验收

按原赛道布局，从上方 S 弯左侧发车，下方直道放置贴边砖。实际运行固件中的 ARM 控制函数完成模拟。拆分前后顺时针 785 帧、逆时针 759 帧的轨迹、状态、舵角和电机输出逐帧一致。

| 闭环模拟 | 终点停稳 | 右环岛附近累计绕行角幅度 | 车身与砖最小间距 |
| --- | --- | --- | --- |
| 顺时针 | 78.4 秒 | 约 475° | 3.71 cm |
| 逆时针 | 75.8 秒 | 约 471° | 4.30 cm |

旧版顺时针在右岛附近仅转过约 108°，逆时针在 16.71 m 处停住。四个单独弯道用例新旧版本均通过，其中两条平缓 S 弯的平均中心偏差从约 2.0 cm 增到约 2.7 cm。实车已验证 Flash 回读和算法 16 启动，实际入环成功率仍需赛道测试。

本次烧录应用长 173600 字节，CRC32 为 `B6072527`，所用 ELF 的 SHA256 为 `a2f0ce9a6cab34d78d8157c4c5ddfa6d17738ab17d30d47326312858253ea8ed`。重新构建的 ELF 可能因调试路径而有不同哈希。
