# Benchmark Report (2026-02-27)

## Environment
- Host: WSL Ubuntu
- Build: `cmake --build build -j 4`
- Bench tools:
  - `./build/bev_bench_ipc`
  - `./build/bev_bench_pipeline`

## Raw Results

### `bev_bench_ipc`
```
bench_ipc_iterations 200000
latency_ns_p50 100
latency_ns_p95 100
latency_ns_p99 100
drops 0
```

### `bev_bench_pipeline`
```
benchmark synthetic_pipeline
samples 21145
fps 1.79276e+06
stage1_ns_p50 199902
stage1_ns_p95 297903
stage1_ns_p99 318602
stage2_ns_p50 144701
stage2_ns_p95 344003
stage2_ns_p99 381203
stage3_ns_p50 475003
stage3_ns_p95 576604
stage3_ns_p99 579604
e2e_ns_p50 878906
e2e_ns_p95 1056708
e2e_ns_p99 1104308
drops_q1 36635
drops_q2 0
drops_q3 2689
serial_fps 1.64995e+06
serial_e2e_ns_p50 500
serial_e2e_ns_p95 600
serial_e2e_ns_p99 600
```

## Notes
- `bev_bench_ipc` shows low overhead for the in-process SPSC ring benchmark and no drops.
- `bev_bench_pipeline` is synthetic and intentionally overloads queues (`drops_q1`, `drops_q3` > 0).
- Parallel synthetic FPS is higher than serial synthetic FPS in this run, but end-to-end synthetic latency is also higher due queueing under overload.
- This benchmark is useful for regression tracking, not for direct flight performance claims.
