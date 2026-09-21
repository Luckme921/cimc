# weld_seam_sdk 2.2.1 回退快照

这里保存 2026-09-21 修改四类周期拐点逻辑之前的完整可编译源码。`main.cpp`
的 SHA-256 为：

```text
f5b855168af6ac53d8214dd471504e6eae4e8cbc58eb3b230a17016b7729e6ff
```

当前安装目录仍保留 `~/scut_weld_sdk_install/lib/libweld_seam_sdk.so.2.2.1`。
如 2.2.2 现场结果不符合预期，可从本快照重新构建安装；该操作只替换算法软件，
不会启动 ROS 节点或硬件：

```bash
cd ~/x86_ros2_ws/x86_weld_seam_test/backups/2.2.1
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
cmake --install build --prefix "$HOME/scut_weld_sdk_install"
```

完成后重新打开终端并重启焊缝节点。终端应显示：

```text
Weld seam SDK 2.2.1 ready
```

需要恢复 2.2.2 时，在 `~/x86_ros2_ws/x86_weld_seam_test` 执行同样三条构建、
安装命令即可。
