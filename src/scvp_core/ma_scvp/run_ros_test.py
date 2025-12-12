import os
import sys
import time
import shutil
import subprocess


def _ensure_parent(path: str) -> None:
    directory = os.path.dirname(path)
    if directory and not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)


if __name__ == "__main__":
    work_dir = os.environ["WORK_DIR"]
    ma_root = os.path.join(work_dir, "src", "scvp_core", "ma_scvp")
    data_cache_path = os.path.join(ma_root, "data")
    log_cache_path = os.path.join(ma_root, "log")

    if len(sys.argv) < 3:
        raise RuntimeError("Usage: run_ros_test.py <model_name> <checkpoint_path>")

    model = sys.argv[1]
    checkpoint_path = sys.argv[2]

    voxel_file = os.path.join(data_cache_path, f"{model}_voxel.txt")
    view_state_file = os.path.join(data_cache_path, f"{model}_vs.txt")
    ready_flag = os.path.join(log_cache_path, "ready.txt")
    target_log_file = os.path.join(log_cache_path, f"{model}.txt")

    # 清理旧文件，避免读取到上一轮结果
    shutil.rmtree(os.path.join(log_cache_path, model), ignore_errors=True)
    if os.path.exists(target_log_file):
        os.remove(target_log_file)
    if os.path.exists(ready_flag):
        os.remove(ready_flag)
    for stale_file in (voxel_file, view_state_file):
        if os.path.exists(stale_file):
            os.remove(stale_file)

    # 等待 C++ 端输出体素与视角状态
    while not (os.path.isfile(voxel_file) and os.path.isfile(view_state_file)):
        time.sleep(0.1)

    _ensure_parent(target_log_file)

    run_time_root = os.path.join(ma_root, "run_time")
    os.makedirs(run_time_root, exist_ok=True)
    os.makedirs(os.path.join(ma_root, "log"), exist_ok=True)

    # 运行时日志需要完整的子目录结构，例如 hb_models/pcd/...
    runtime_subdir = os.path.join(run_time_root, os.path.dirname(model))
    if runtime_subdir:
        os.makedirs(runtime_subdir, exist_ok=True)

    subprocess.run(
        [
            "python3",
            os.path.join(ma_root, "infer.py"),
            checkpoint_path,
            model,
        ],
        cwd=ma_root,
        check=True,
    )

    # 创建 ready.txt 用来通知 C++ 端
    os.makedirs(log_cache_path, exist_ok=True)
    with open(ready_flag, "w", encoding="utf-8"):
        pass
