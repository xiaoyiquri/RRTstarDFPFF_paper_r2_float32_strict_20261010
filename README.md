# paper_r2 float32 strict 新副本

本目录由 `RRTstarDFPFF_paper_r2_noguidance_20261006` 复制建立，原目录保持不变。

主模式为 `flask_strict` / `float32_production`：无人工地图 Guidance，r=2 cubic PFF，float 状态、代价、NN、DF 和 AVX2 时间采样检查；保留 sphere union。2026-10-09 起默认 `time_policy: optimal_only`、额外时间采样 0、shortcut 关闭，先测搜索首解性能。开启 shortcut 后仍分别保存 raw、shortcut、首解及边界诊断。

A′ 语义保持不变：y、dy 连接连续，Quad 姿态和 omega 只诊断。完整物理连续性及连续碰撞没有被认证。单根固定 Unicycle signed branch；未增加双 branch 森林。

- [具体实现与数学公式](docs/float32_strict_implementation.md)
- [Ubuntu 构建、运行及可视化完整命令](运行指令.md)
- [修改文件与交付范围](修改说明.md)
- [本机验证记录](docs/verification_report.md)
- [本轮 latency / anytime 分离与 T*-only 修改](docs/latency_anytime_tstar_20261009.md)
- [原目录完整性核验](docs/original_integrity_check.json)

主目录六个 YAML 为 latency：`stop=true`、首解前不 rewire；`benchmark/config/anytime` 为独立 anytime preset：`stop=false`、首解前 rewire。`tools/run_six.py --protocol` 选择对应配置，两组默认都是 20 seeds、120 s、同系统相同 planner 参数。Uni 两张图的 range 统一为 0.30。

`--time-policy candidate_set --additional-time-samples 0` 保留根及时间上下界的多候选消融；`optimal_only` 只把最低代价 T 送去验证，失败后不尝试其他时长。`--shortcut true --shortcut-mode flask --shortcut-time optimal` 可独立开启后处理。

`full_df` 继续使用 `legacy_mixed`；`double_reference` 仅在测试中使用。旧模式的配置和求解链保持原样。历史文档在 docs/history，现行语义以本 README 和实施说明为准。
