# RT1064 烧录工具 · Windows x64

使用 Python 3.13 x64、PowerShell 7 和项目的 Git 仓库。在本目录执行：

~~~powershell
pwsh -File ./install.ps1
~~~

默认安装到当前用户的 LocalAppData/Programs/RT1064Tools。仓库中的安装脚本从 PyPI 获取锁定的依赖；随离线版提供的 wheels 目录适用于 CPython 3.13 x64。下载器为逐飞 SEEKFREE CMSIS-DAP，pyOCD 内置 mimxrt1064 目标支持。可先运行下面的只读命令：

~~~powershell
& "$env:LOCALAPPDATA/Programs/RT1064Tools/venv/Scripts/pyocd.exe" list
~~~

在项目根目录运行 ./build.ps1，再运行 ./flash.ps1。入口会备份扇区，调用原始 flash-rt1064.ps1 写入组合 HEX，再逐区回读并检查启动。pyocd.yaml 由 Project/tools/wired-update.py 临时生成，使用 under-reset、800 kHz 和按扇区擦除。

接线时先断电。电脑 USB 接下载器，原配 SWD 排线接主板调试口并对齐 1 脚。若用散线，按两端丝印连接 GND、SWCLK、SWDIO 和 nRESET。具体针序需对照实物。目标电压参考脚按下载器说明接，先辨明 VTref 输入与 3V3 电源输出。烧录时主板需独立正常供电、核心板 CORE 灯亮；单独烧录可以断开电机动力电源。若车上主板由驱动板稳压输出供电，需保留主板供电路径并隔离电机负载。未记录现场电池电压、主板稳压路径和驱动板动力端的实测值。

* [Python Windows 安装包](https://www.python.org/downloads/windows/)
* [pyOCD 安装说明](https://pyocd.io/docs/installing)
* [逐飞产品目录](https://www.seekfree.cn/products)
