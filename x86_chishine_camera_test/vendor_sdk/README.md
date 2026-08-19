# vendor_sdk 说明

此目录是原厂 `3DCameraSDK-v3.2.52.20240412` 的 Linux x86_64 最小运行子集：

- `inc/3dcamera/`：相机、帧、系统、点云重建所需公开头文件；
- `lib/3dcamera/linux/x64/lib3DCamera.so`：x86_64 原厂运行库。

没有修改原厂头文件或动态库。当前 `lib3DCamera.so` 大小为 `94852864` 字节，复制时校验的 SHA-256 为：

```text
B916D54E21B80E3136215600AE2AA92A050B41E7BFC1B5ADB34CBC58383D4F3D
```

部署到 Ubuntu 后必须运行：

```bash
file lib/3dcamera/linux/x64/lib3DCamera.so
ldd lib/3dcamera/linux/x64/lib3DCamera.so
```

本目录不能替代原厂完整 SDK 归档。升级相机 SDK 时应整体更换匹配版本的头文件和 `.so`，重新独立测试并记录新哈希；不要只换动态库或只换头文件。授权限制见上级 `THIRD_PARTY_NOTICES.md`。
