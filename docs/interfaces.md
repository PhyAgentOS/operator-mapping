# 接口

输入：tick、finish、policy_command（PolicyCommand，policy_id=mapping）。命令：start / save / stop。输出：status（PolicyCommandStatus，保留 request_id）。

ROS2 的 LaserScan、Odometry、OccupancyGrid、Path、TF 在后端交换，不新增 Forge 消息类型。具体配置键见示例 YAML；topic、ROS domain、程序路径由调用方指定。

## 最小调用

```python
from forge_msgs import PolicyCommand

request = PolicyCommand.from_inputs(
    policy_id="mapping", command="start", request_id="example-1", inputs={}
)
node.send_output("policy_command", request.to_arrow())
```

每次新任务使用唯一 request_id；重复相同请求返回已有状态，重复 ID 配不同参数被拒绝。status 常见值为 accepted/running/done/rejected/error；done 是此命令完成，不表示后端进程退出。

| command | inputs 示例 | 作用 |
|---|---|---|
| `start` | `{}` | 启动 SLAM Toolbox，首张地图后 done |
| `save` | `{}` | 保存到 session_dir/archives，返回地图引用和文件清单 |
| `stop` | `{}` | 停止本节点启动的建图后端 |

`examples/dataflow.yaml` 展示完整输入输出连接。dynamic 节点是调用方占位，不会凭空提供数据；实际应用需用自己的节点替换。`tick` 为调度输入，`finish` 任意负载触发退出。所有进程在同一机器人运行，统一 ROS_DOMAIN_ID。
