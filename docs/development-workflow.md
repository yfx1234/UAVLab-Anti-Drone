# 修改与仓库同步

## 一次改动的闭环

1. 从 origin 更新，检查本地改动与板上/电脑运行版本；不覆盖未纳入本次任务的用户文件。
2. 在仓库修改；验证相关逻辑。电脑自动测试用 `python -m unittest discover -s tests -v`，部署包用感知 CTest；硬件测试另行记录条件和结果。
3. 需部署时先释放相机并备份。按 `scripts/stage_board.py` 复制源码，构建后再由面板启动；仅文档或仓库准备无需打断现有采集。
4. 检查 diff、新增文件与 `scripts/check_repository.py --history`。确认模型来源/哈希更新，不上传密码、密钥、视频、引擎、缓存或整机镜像。
5. 明确暂存本次修改、commit、push，确认远端分支 SHA 与本地 HEAD 一致，记录部署版本。没有推送成功就报告尚未同步，不能把本地提交当成已上 GitHub。

用户已授权此项目后续每次改动同步 GitHub，无需重复确认。推送认证使用 GitHub CLI/Windows 凭据管理，不将令牌写入项目。禁止强制推送和无选择地提交其他任务文件。

## 什么进仓库

源码、依赖说明、相机适配源/参考配置、当前实测环境、测试、迁移步骤、已登记模型PT/ONNX。模型体积增长或频繁训练时再改为 Git LFS/Release，并保留哈希索引。

录制 SVO/视频、训练数据集、运行日志、私钥、密码、GitHub令牌、SDK/驱动安装包和构建产物留在本机/板子。当前 Nano 的引擎清单只记录来源、张量和哈希；NX引擎重建后更新清单。

`environment/hardware/current-nano` 为现机参考，不自动应用到新核心。仓库保留早期提交，默认构建及日常界面只使用当前感知链路。

## 人工同步命令

```bash
git diff --check
python scripts/check_repository.py --history
git add <本次修改的文件>
git commit -m "Describe the verified change"
git push
git ls-remote origin HEAD
```

这是修改完成后的同步约定，不是监视文件系统并自动上传每一次未完成编辑。
