# 本轮验证记录（2026-10-09）

Release 重新编译成功，16/16 C++ 测试与 9/9 Python 测试通过。
两个非法 production YAML（未知 time_policy、optimal_only 搭配非零额外采样）均返回 1 并给出明确错误。

| 检查 | 每场景预算 / seed | sampled 成功数 | 输出记录检查 |
|---|---|---|---|
| latency_candidate_zero | 3.0 s / 41 | 4/6 | passed |
| latency_optimal_only | 3.0 s / 41 | 4/6 | passed |
| anytime_optimal_only | 2.0 s / 41 | 4/6 | passed |
| shortcut_optimal | 3.0 s / 41 | 0/1 | passed |
| shortcut_quad2d | 3.0 s / 41 | 1/1 | passed |

六环境三组检查均为四个 Quad 成功、两个 Unicycle timeout_no_goal。latency 在首解终止且 rewires=0；anytime 持续到 timeout，有 rewiring 和代价事件。
短时失败如实计入 summary，未调整 branch / Near / 几何 / A′ / 模型来消除失败。不能据此推断 120 s 或 20 seeds 的成功率，也不据此宣称性能提升。

成功的 Quad2D Hole 独立启用 shortcut=true、flask、optimal，输出 raw / shortcut 并通过记录检查。
本轮官方 DynoBench 绘图检查生成 latency 4 张、anytime 4 张、shortcut 场景 1 张 PNG；失败的 Unicycle 未生成轨迹图。

原 noguidance 目录核验 219 个文件，changed_or_missing=[]、added=[]、original_unchanged=true。
binary SHA256: `a181fbca7f680c8457d8df035fcfc0563978057444540ebf4c20d09faf7116f8`

## 本轮结果和图片

以上实验均使用 build_float32、单核 affinity [0]、float32_production、sphere_union，均不是正式 20 seeds 实验。
- [latency_candidate_zero](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/latency_candidate_zero/summary.csv)
- [latency_optimal_only](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/latency_optimal_only/summary.csv)
- [anytime_optimal_only](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/anytime_optimal_only/summary.csv)
- [shortcut_optimal](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/shortcut_optimal/summary.csv)
- [shortcut_quad2d](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/shortcut_quad2d/summary.csv)
- [latency_optimal_only 官方图片索引](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/latency_optimal_only/figures_official_raw/index.html)
- [anytime_optimal_only 官方图片索引](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/anytime_optimal_only/figures_official_raw/index.html)
- [shortcut_quad2d 官方图片索引](D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_float32_strict_20261007/results/protocol_tstar_validation_20261009_225317/shortcut_quad2d/figures_official_raw/index.html)

## 文件清单

- pff/r2_float_kernel.h
- benchmark/production_runner.cpp
- tools/run_six.py
- tools/verify_results.py
- tests/test_float32.cpp
- tests/test_paper_profile.cpp
- tests/test_run_six.py
- README.md
- 修改说明.md
- 运行指令.md
- docs/float32_strict_implementation.md
- docs/latency_anytime_tstar_20261009.md
- benchmark/config/uni_bugtrap.yaml
- benchmark/config/anytime/uni_bugtrap.yaml
- benchmark/config/uni_wall.yaml
- benchmark/config/anytime/uni_wall.yaml
- benchmark/config/quad2d_bugtrap.yaml
- benchmark/config/anytime/quad2d_bugtrap.yaml
- benchmark/config/quad2d_hole.yaml
- benchmark/config/anytime/quad2d_hole.yaml
- benchmark/config/quad3d_block.yaml
- benchmark/config/anytime/quad3d_block.yaml
- benchmark/config/quad3d_window.yaml
- benchmark/config/anytime/quad3d_window.yaml
- summary.md
- 交付说明.md
