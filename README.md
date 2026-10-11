# MyTessNG

基于 **TESS NG** 微观交通仿真软件 Python API 的二次开发示例项目，实现自定义交通仿真插件与信控仿真逻辑。

## 项目简介

本项目通过 TESS NG 提供的 Python 接口（`Tessng` 模块）开发一个自定义插件，在仿真过程中实现自定义的车辆控制、信号配时与路网逻辑。适用于交通工程、信号控制、车路协同等方向的研究与教学。

## 环境要求

- Python 3.6+（推荐与 TESS NG 内置 Python 版本一致）
- TESS NG V4.1（需自行安装，本项目不含软件本体）
- 依赖见 `requirements.txt`

```bash
pip install -r requirements.txt
```

主要依赖：`PySide2==5.15.2.1`、`numpy`、`pandas`、`matplotlib`、`ortools`、`torch`、`sko` 等。

## 项目结构

```
.
├── main.py                    # 程序入口，初始化 QApplication 与 Tessng 工厂
├── MyPlugin.py                # 自定义插件定义（继承 TESS NG 插件基类）
├── MySimulator.py             # 核心仿真逻辑（自定义跟驰/换道/信控等）
├── MyNet.py                   # 自定义路网逻辑
├── DockWidget.py              # 自定义 UI 停靠面板
├── config.py                  # 配置参数
├── function.py                # 工具函数
├── TESS_API_EXAMPLE.py        # TESS NG API 官方示例参考
├── main.spec                  # PyInstaller 打包配置
├── requirements.txt           # Python 依赖
└── 机动车交叉口.tess           # 示例路网文件（机动车交叉口场景）
```

## 运行方式

**注意**：本项目依赖 TESS NG 运行时环境（`Tessng.pyd` 及相关 dll），需在已安装 TESS NG 的环境下运行。

1. 安装 TESS NG V4.1 软件；
2. 将其 Python API 相关模块（`Tessng.pyd`、各 `TESS_*.dll` 等）放入运行目录；
3. 安装 Python 依赖；
4. 运行入口：

```bash
python main.py
```

程序将加载 `机动车交叉口.tess` 路网并启动仿真。

## 说明

- 本仓库仅包含二次开发的**源码与文档**，不包含 TESS NG 软件本体、安装包、授权证书及运行时 dll。
- 使用前请确保已通过正规渠道获得 TESS NG 软件授权。

## 许可证

本项目源码仅供学习与交流使用。
