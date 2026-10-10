# latency / anytime 分离与 T*-only（2026-10-09）

本轮按用户附件修改现有 float32 strict 副本。较早的 noguidance 原目录和已有结果目录保持原样。

## 配置与实验目的

| 项目 | latency：benchmark/config/*.yaml | anytime：benchmark/config/anytime/*.yaml |
|---|---|---|
| stop_at_first_solution | true | false |
| rewire_before_first_solution | false | true |
| timeout | 120 s 失败保护 | 每 seed 120 s 优化 |
| time_policy / additional_time_samples | optimal_only / 0 | optimal_only / 0 |
| shortcut | false | false |
| seeds | 41..60 | 41..60 |
| 观察指标 | first_solution_time、first_solution_cost、成功率 | anytime_N.csv 的 J_best(t)、首解、raw_cost |

协议之间只有两个 planner 开关不同；anytime 相对 source/model/output 路径因目录深度不同而调整，仍使用相同数据和模型。
Uni 两图固定 goal_bias=0.15、range=0.30、rewire_factor=3、goal_parent_limit=16、minimum_parent_candidates=8、goal_tolerance=1e-6。
Quad2D、Quad3D 每个系统的两图也使用相同 planner 参数。Quad3D 将原默认生效的 cache_goal_connections=true、minimum_parent_candidates=8 写明。
未改变 branch、Near、collision resolution、sphere geometry、A′、robot model、PFF min/max/fixed_time 或 R。

## 时间策略

`FloatPFFConfig` 新增 FloatTimePolicy，C++ 默认 CandidateSet；缺少 YAML time_policy 时仍解析为 candidate_set。
`production_runner.cpp` 解析两个合法字符串并拒绝其他值；optimal_only 与非零额外采样组合在配置解析和内核构造时都拒绝。

1. 使用原 float 求根器取得区间内驻点，加上 min_time、max_time。
2. 用真实边界闭式 J(T) 计算并稳定排序；代价相同按较小 T 排序。
3. optimal_only 截断为一项，只将该 T 送到 DF / 约束 / SIMD collision；失败不回退到其他时长。
4. candidate_set 返回完整候选集；额外均匀采样仅在该模式允许。

该策略同样作用于 scalar、AVX2 batch、expansion、parent selection、goal、rewire，以及 time_mode=optimal 的 shortcut。
上下界参与选择，T* 指现有 float 数值求根和边界比较得到的区间最优候选，不宣称精确代数根或物理可行时间的最优解。
固定时间求解保留 fixed_time 语义。关闭 shortcut 后不进行 Algorithm 4 连接；显式开启仍保存 raw / shortcut。

## 脚本与记录

tools/run_six.py 按 --protocol 选择主目录或 anytime preset，显式设置对应的两个 planner 开关。
默认 --shortcut false、--shortcut-time optimal；生产模式默认 optimal_only / 0。
新增 --time-policy 和 --additional-time-samples，支持同 seed 的 candidate_set / 0 与 optimal_only / 0 对照。
candidate_set / 12 仅作额外消融；legacy 不接受这些新覆盖参数，旧 YAML 和计算链保持原样。
manifest 保存策略、额外采样数、首解前 rewiring、实际 preset 路径、生成配置哈希和实际 planner 开关。
tools/verify_results.py 对新 manifest 检查生成配置与记录一致，同时保留旧生产结果兼容检查。

运行和绘图完整命令见根目录运行指令.md。latency 的 search_time 不能直接与 anytime 的 120 s search_time 比较；
J_best(t) 只描述原树搜索过程，shortcut 后代价另行统计。

## 验证

本轮日志保存在 docs/validation_20261009/。C++ 检查自由/固定终端速度、时间上下界最优、固定时间、延迟构造、
AVX/scalar 单候选一致性和非法组合；配置检查协议差异、同系统统一参数及模型路径一致。
Python 检查协议编排、默认参数、两种 candidate_set 消融、可选 shortcut、legacy 兼容及非法参数拒绝。
短时六环境结果只验证运行链，不代表 20 seeds 正式统计。最终实际测试结论写入该目录 report.md。
