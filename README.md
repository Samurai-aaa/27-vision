# 27-vision-&lt;name&gt;

> RoboMaster 装甲板检测 · 华南虎视觉组夏令营任务
>
> ⚠️ 仓库名暂用占位 `27-vision-dev`，注册 GitHub/Gitee 私有仓库后请改为 `27-vision-<name>`。

## 任务清单

| 目录 | 任务 | 状态 |
|---|---|---|
| [task1_armor_detection](task1_armor_detection/) | 装甲板检测：传统 OpenCV + RP24 神经网络 | ✅ 已完成 |
| [task2_xxx](task2_xxx/) | 待定 | 🚧 占位 |
| [task3_xxx](task3_xxx/) | 待定 | 🚧 占位 |

## 目录结构

```
27-vision-<name>/
├── README.md                    # 本文件
├── CHANGELOG.md                 # 顶层版本记录
├── docs/                        # 任务报告（导出 PDF 放这里）
├── task1_armor_detection/       # 任务1：装甲板检测
│   ├── traditional_opencv/      #   子方案A：传统 OpenCV 算法
│   └── rp24_nn/                 #   子方案B：RP24 神经网络部署
├── task2_xxx/                   # 任务2（占位）
└── task3_xxx/                   # 任务3（占位）
```

## 各任务说明与编译运行

每个任务文件夹内部自成体系，含任务代码、`CMakeLists.txt`、运行效果视频、README（编译运行步骤）与 CHANGELOG：

- **task1**：`task1_armor_detection/README.md`，两个子方案各带自己的编译方式
  - 传统算法：`task1_armor_detection/traditional_opencv/Readme.md`
  - 神经网络：`task1_armor_detection/rp24_nn/README.md`

## 提交约定

按夏令营要求，最终以压缩包提交：`<姓名>-<方向>-华南虎视觉组夏令营任务.zip`，内含：

1. 任务报告导出 PDF（`docs/`）
2. 每个任务一个文件夹（代码 + CMakeLists + 效果视频 + README + CHANGELOG + 本地 git）
