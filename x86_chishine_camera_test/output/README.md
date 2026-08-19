# output 目录

用于保存 `chishine_camera_test --capture` 生成的测试 PLY。建议文件名包含日期、相机序列号、拍照位和曝光配置。确认文件非空并能被焊缝 CLI 读取后，再测试 ROS 2 相机节点。

这是运行数据目录，不包含相机标定或原厂 SDK 配置；删除 PLY 不影响重新编译。
