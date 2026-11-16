# 二维建图

管理 SLAM Toolbox 建图、地图输出和保存；扫描、里程计与 TF 在 ROS 2 后端之间传递。

## 运行要求

- Node 版本：`0.2.0`；兼容 **Forge 1**。
- Linux x86_64、Ubuntu 24.04、Python 3.12 ABI、ROS 2 Jazzy。
- Dora CLI / Python 绑定 `0.4.1`、Forge Msgs `1.3.0`、Forge Common `1.0.1`。
- ROS 2、算法后端、Livox 消息工作空间由部署方提供，不包含在 Node 包内。启动前加载对应 ROS setup，运行 `--check-runtime` 检查环境；该命令不连接硬件。

制品是 `directory_tar_gz`，解压后保留整个 `mapping/` 目录，入口为 `mapping/mapping`。冻结包包含 Dora Python 绑定，不能只升级外部 Dora CLI 来切换到 Forge 2。

## 能力边界与可选配置

- `backend_overrides` 在会话目录生成参数副本，原始参数文件不变。
- 可选 `snapshot_dir` 输出地图和位姿观测。地图的用户命名、选择、导入导出界面由调用方负责。
- 本 Node 不控制底盘探索，不管理 SSH 或跨节点工作流。

这些能力通过配置启用，未配置的可选功能保持关闭。Node 仅拥有自己的后端子进程，不接管其他应用进程或写入机器人厂家工程。快照是运行观测，不是控制接口；读取方负责检查时效。

## 开发与构建

```bash
uv sync --group dev
uv run mapping --check-config --config config/policy.example.yaml
uv run pytest
uv run ruff check .
uv pip install --python .venv/bin/python -r requirements-build.txt
source /opt/ros/jazzy/setup.bash
# 按部署环境加载所需消息及算法工作空间。
BUILD_PYTHON=.venv/bin/python bash scripts/build_pyinstaller.sh
```

配置示例中的路径和话题需由使用者填入；实际执行应由 Dora dataflow 编排。构建时使用与测试相同的依赖环境。`--version`、配置检查和单元测试不替代 ROS 后端及真机验收。

接口说明见 [docs/interfaces.md](docs/interfaces.md)，配置在 `config/`，实现位于 `src/`，测试位于 `tests/`。构建工具、测试及源码不随冻结运行制品提供。
