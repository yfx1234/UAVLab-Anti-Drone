---
name: "UAVLab 感知与采集工作台"
description: "以真实画面为中心、无外围描边与轻浮起圆角表面的稳定采集软件。"
colors:
  primary: "#2a2a2e"
  primary-hover: "#151517"
  background: "#f2f2f4"
  paper: "#fff"
  rail: "#f6f6f8"
  ink: "#1d1d1f"
  ink-secondary: "#3d3d42"
  muted: "#6b6b70"
  faint: "#75757a"
  line: "#ececef"
  control-fill: "#ececf0"
  placeholder: "#6b6b70"
  image: "#1c1c1e"
  success: "#1c7550"
  success-dot: "#2f9f69"
  danger: "#b2333b"
  danger-dot: "#e0484f"
  warning: "#87590a"
  warning-dot: "#dc9b2a"
  warning-surface: "#fbf1dc"
  hover: "#e2e2e7"
  pressed: "#d8d8de"
  danger-surface: "#fcecec"
  danger-hover: "#f8dfe0"
  disabled-surface: "#f4f4f6"
  disabled-text: "#a2a2a7"
typography:
  brand:
    fontFamily: "\"Segoe UI Variable Text\", \"Segoe UI\", -apple-system, BlinkMacSystemFont, \"PingFang SC\", \"Noto Sans SC\", \"Microsoft YaHei UI\", \"Microsoft YaHei\", sans-serif"
    fontSize: "18px"
    fontWeight: 650
    lineHeight: 1.2
    letterSpacing: "-0.01em"
  headline:
    fontSize: "16px"
    fontWeight: 600
    lineHeight: 1.35
    letterSpacing: "0"
  control-title:
    fontSize: "14px"
    fontWeight: 600
    lineHeight: 1.4
    letterSpacing: "0"
  body:
    fontFamily: "\"Segoe UI Variable Text\", \"Segoe UI\", -apple-system, BlinkMacSystemFont, \"PingFang SC\", \"Noto Sans SC\", \"Microsoft YaHei UI\", \"Microsoft YaHei\", sans-serif"
    fontSize: "14px"
    fontWeight: 400
    lineHeight: 1.55
  control:
    fontSize: "13px"
    fontWeight: 500
    lineHeight: 1.4
  label:
    fontSize: "13px"
    fontWeight: 400
    lineHeight: 1.4
  supporting:
    fontSize: "12px"
    fontWeight: 400
    lineHeight: 1.5
  metric:
    fontSize: "18px"
    fontWeight: 600
    lineHeight: 1.3
  mono:
    fontFamily: "\"Cascadia Mono\", ui-monospace, \"SF Mono\", Menlo, Consolas, monospace"
    fontSize: "12px"
    fontWeight: 400
    lineHeight: 1.6
rounded:
  pill: "999px"
  field: "10px"
  feedback: "16px"
  panel: "18px"
  surface: "24px"
  dialog: "28px"
spacing:
  tight: "4px"
  inset: "8px"
  compact: "12px"
  standard: "16px"
  section: "20px"
  workspace: "24px"
components:
  button-primary:
    backgroundColor: "{colors.primary}"
    textColor: "{colors.paper}"
    rounded: "{rounded.pill}"
    padding: "6px 14px"
  button-primary-hover:
    backgroundColor: "{colors.primary-hover}"
  button-secondary:
    backgroundColor: "{colors.control-fill}"
    textColor: "{colors.ink}"
    rounded: "{rounded.pill}"
    padding: "6px 14px"
  button-danger:
    backgroundColor: "{colors.danger-surface}"
    textColor: "{colors.danger}"
    rounded: "{rounded.pill}"
    padding: "6px 14px"
  button-danger-fill:
    backgroundColor: "{colors.danger}"
    textColor: "{colors.paper}"
    rounded: "{rounded.pill}"
    padding: "6px 14px"
  button-disabled:
    backgroundColor: "{colors.disabled-surface}"
    textColor: "{colors.disabled-text}"
    rounded: "{rounded.pill}"
  input:
    backgroundColor: "{colors.control-fill}"
    textColor: "{colors.ink}"
    rounded: "{rounded.field}"
    padding: "6px 12px"
  status-chip:
    backgroundColor: "{colors.paper}"
    textColor: "{colors.ink-secondary}"
    rounded: "{rounded.pill}"
    padding: "0 11px 0 10px"
    typography: "{typography.supporting}"
  workspace-tab:
    textColor: "{colors.muted}"
    rounded: "{rounded.pill}"
    padding: "5px 16px"
    typography: "{typography.control}"
  workspace-tab-selected:
    backgroundColor: "{colors.paper}"
    textColor: "{colors.ink}"
  section:
    backgroundColor: "{colors.paper}"
    textColor: "{colors.ink}"
    rounded: "{rounded.surface}"
---

# Design System: UAVLab 感知与采集工作台

## Overview

**Creative North Star: "稳定的软件工作台"**

以真实画面为中心的成熟采集软件。柔和中性工具面、石墨主操作、胶囊按钮与轻浮起的大圆角表面让控制保持清楚；常驻表面和控件不加外围描边。UAVLab 品牌和中文操作语言沿用。系统 UI 字体服务连续操作，测量数据用等宽数字对齐。

实时数据不改变操作布局。导航、控制、设备健康与反馈有稳定的位置；工作区切换只改变前端视图，相机、YOLO、录像与后台下载的生命周期独立。视觉层级由空间、字重、选中状态和真实图像建立，不依靠装饰标题或重复卡片。

**Key Characteristics:**

- 中性工具面、石墨主要操作与无外围描边的轻浮起表面。
- 小按钮两端半圆，大容器按尺度使用柔和圆角。
- 真实画面为中心，控制和状态保持稳定。
- 中文系统 UI 排版：五级字号、无中文负字距、信息文字不小于 12px，测量数字对齐。
- 状态以中性文字配彩色圆点表达；只有注意与异常才使用浅色底。
- 完整的焦点、禁用、错误、确认与可恢复反馈。

本文据 `panel/index.html` 当前实现合并既有有效规则；用户已确认无外围描边、胶囊按钮、大圆角与轻微浮起效果。2026-10-21 第二轮按用户“文字呈现不好、可以自由发挥”的要求重做排版与状态语法：页头合为单行、控制栏改为“标签｜控件”字段行、测量改为指标块、目标表按列拆分、就绪检查圆点对齐。旧版永久边框、小圆角、蓝色主操作、全场 13px、22/17/15px 混用标题、11px 信息文字、中文负字距、彩色状态文字刷屏与斜杠合并的测量值不再作为视觉规范。具体页面故事与首屏构图留在 `.impeccable/surface-briefs/console.md`。

## Colors

柔和浅灰工具面配石墨主要操作；语义色只补充运行、故障与测量质量。frontmatter 是色值来源。sidecar 的八阶 OKLCH 色条仅供面板比较色调，为合成预览，不扩展生产色板。

### Primary

- **石墨操作色**：启动、主要下载、进度与固定反馈；源码变量为 `--primary`（旧 `--blue` 已移除）。
- **深石墨悬停色**：主要按钮悬停。

### Neutral

- **工作台底色 / 白色内容面 / 浅灰工具面**：页面、图像与表格容器、工具条和设备健康区。
- **墨色 / 次级墨色**：标题与数值用墨色；表格与说明正文用次级墨色。
- **辅助灰 / 浅辅助灰**：字段标签、状态说明与表头用辅助灰；表头单位、未到阶段与空闲值用浅辅助灰。两者在白色面上对比度分别约 5.3:1 与 4.6:1，均满足小字 4.5:1；浅辅助灰只用于白色面。
- **分隔线**：表格行、检查行与分组之间的 1px 内部分隔，不作外围描边。
- **控件浅灰 / 占位文字灰**：输入、选择与次级操作的填色；placeholder 保持完整可读。
- **图像石墨**：监看和回放底面。
- **悬停灰 / 按下灰**：次级操作的短状态反馈；工作区选中项使用白色面与轻阴影。
- **危险浅红 / 危险悬停浅红**：停止与删除入口的低面积语义填色。
- **禁用底色 / 禁用文字**：不可用控件比可用控件更浅、文字更淡，一眼可分；不降低整个区域的透明度。

运行绿、故障红与警示棕分别对应 success、danger、warning，每种另有更亮的圆点色（success-dot、danger-dot、warning-dot）；警示与故障另有浅底（warning-surface、danger-surface）。保留实际状态文字。图像内 YOLO 绿色框为数据叠加，不是界面品牌色。

**The Semantic Color Rule.** 运行、异常与未标定状态同时使用文字和语义色；正常运行用中性文字加绿色圆点，注意与异常的文字才着色。选择与主操作使用中性石墨，不恢复旧版蓝色主操作。

## Typography

**Body Font:** 使用源码 `--font-ui` 的系统 UI 栈，具体顺序见 frontmatter。Windows 11 上西文与数字实际由 Segoe UI Variable 渲染，中文由已安装的 Noto Sans SC 渲染（2026-10-21 在用户电脑上用 Chrome 的 CSS.getPlatformFontsForNode 核实）；未安装 Noto Sans SC 时回退 Microsoft YaHei UI。Apple 平台字体名仅是跨平台回退项。
**Display Font:** 无独立装饰 display 字体。
**Label/Mono Font:** 标签沿用正文；测量、计时与健康数值使用 `font-variant-numeric: tabular-nums`。只有本机文件路径使用 `--font-mono`（Cascadia Mono 起始的等宽栈），正文不改成等宽字体。

### Hierarchy

- **Brand**：UAVLab 名称 18px/650，是唯一带 -0.01em 字距的文字；窄屏 17px。
- **Headline**：内容区标题（实时画面、目标与相机坐标、拦截任务、板上场次、本机视频）统一 16px/600。
- **Control title**：控制分组与子区标题（设备连接、网枪发射、就绪检查、下载到电脑）14px/600。
- **Body**：正文 14px/1.55。
- **Control / Label**：按钮、输入、字段标签、检查行与表格内容 13px；按钮字重 500。
- **Supporting**：状态胶囊、表头、单位、规格、页头时钟、辅助说明 12px；不再使用 11px 承载信息。
- **Metric**：测量指标数值 18px/600，单位 12px 辅助灰；设备健康数值 15px/600。

模态标题与图像空状态标题为局部功能层级，不扩展为营销标题尺度。实际字号见 frontmatter。

**The Operational Type Rule.** 沿用系统 UI 字体与实际功能层级，不引入装饰性 display 字体；中文标题不加负字距，信息文字不小于 12px；数值使用 tabular-nums，正文不改成技术等宽字体。

## Layout

桌面主容器最大宽度 1820px，水平留白 24px，顶部留白 8px；设备健康网格同步为 1772px。实时控制栏与素材来源栏均为 264px，1500px 以上为 272px，工作区间距 20px。工具栏 sticky，设备健康栏 fixed；实际高度经 ResizeObserver 回写变量，让内容、控制和反馈避开遮挡。

页头为单行工具条：品牌与副标题（中间 1px 分隔）、工作区分段切换、右侧状态胶囊（连接、时钟 | 相机、录像、拦截链）、后台任务指示与北京时间。1100px 以下隐藏副标题、时钟同步胶囊与任务指示，使 1024px 仍为单行（约 950px 以下且状态文字较长时，状态行换到第二行）；599px 以下状态行占满宽度并隐藏相机与拦截链胶囊。

实时监看是带命名区域（controls / live / mission / targets）的网格，间距 20px。1400px 以上为三列：控制栏、画面与目标表、拦截任务栏 300px（1500px 以上 316px）。1101–1399px 为两列，拦截任务栏位于画面与目标表之间并占满右列；1100px 以下单列，按控制、画面、拦截任务、目标表顺序堆叠。DOM 顺序与窄屏阅读顺序一致。

桌面控制与素材来源栏 sticky 于工具栏下方 8px，最大高度扣除工具栏、健康栏与 24px 留白，内部滚动；素材来源栏按 100dvh 计算可用高度。用户的典型视口 2048×1010 下，控制栏四个分组（含板上录像）无需滚动即可完整显示。三列模式下拦截任务栏同样 sticky 并内部滚动，两列与单列时回到文档流。1100px 以下实时控制可折叠，展开后各分组以 CSS 多列（最多 3 列、每列至少 240px、分组不拆分）平衡排布，避免行对齐留下大块空白；599px 以下为单列。素材库在 800px 以下改为单列，来源栏恢复正常文档流与自然高度。健康指标由十列减至五列、两列；所有窗口默认折叠，手动展开后调整窗口不覆盖用户选择，保持可查。

目标表外容器固定 264px，sticky 表头、双向内部滚动与 stable scrollbar gutter；测量数据不换行。目标区是 `targets` 尺寸容器：宽度小于 880px 时单元格内边距收为 6px，使 1400–1599px 三列与 1101–1399px 两列下 12 列仍能完整显示。实时监看明确占满容器宽度、采用 16:9，保留最大高度 650px、默认最小高度 200px、窄屏 180px；原生全屏保持画面比例。

本机回放整个卡片按窗口可用高度等比缩小：源码 `--media-room` 为 `max(200px,calc(100dvh - var(--header-height) - var(--health-height) - 228px))`；卡片宽度为 100%，最大宽度为该高度乘 16/9。素材库整个工作区以来源栏宽度、间距和该媒体宽度限制最大宽度，通过水平自动外边距整体居中，让两侧留白对称；不单独居中视频而把场次栏留在页面边缘。标题区、选择字段、视频与文件信息共同遵循该卡片宽度。800px 以下移除工作区和卡片最大宽度，随自然堆叠占满所在列。

回放 stage 明确为卡片的 100% 宽，由 16:9 决定高度，最小高度为 0，不设 650px 高度上限。视频绝对定位填满 stage，使用 object-fit: contain 保留完整画面；不得通过单独缩小视频、裁切或拉伸消除右侧空白。文件信息在正常文档流中。

**The Stable Targets Rule.** 目标区保持固定高度、sticky 表头和内部滚动；更新复用行节点并保留双向滚动位置，不随目标数量移动外部布局。

**The Independent Workspaces Rule.** 切换实时监看与素材库只切换视图，不发送相机或录像启停 RPC；设备健康跨工作区可查。

**The Reachable Fire Rule.** 网枪发射入口位于拦截任务栏的固定位置，宽屏与实时画面同屏可见，窄屏紧随画面；不放进需要滚动的控制栏底部。发射为单发且不可撤回，必须经过确认对话框；下载或标注占用面板时，停止拦截链与兜底发射仍可用。

## Elevation & Depth

常驻区域依靠浅灰底色、白色内容面、石墨图像面与柔和环境阴影产生层次。控制栏、素材来源栏和内容容器使用同一低幅表面阴影；工作区选中项、图像分段按钮与页头状态胶囊用更小的阴影表示浮起。设备健康栏由向上阴影与内容分离；反馈固定覆盖显示，不推移内容。模态确认有柔和阴影与半透明背景。精确值见 sidecar extensions.shadows。

**The Quiet Surfaces Rule.** 常驻表面以填色、间距和轻微浮起阴影区分，不加外围描边；目标表与检查列表的内部行分隔与键盘焦点轮廓保留。

## Shapes

圆角按对象尺度使用：按钮、工作区导航、分段切换与状态胶囊使用胶囊角色，两端呈半圆；输入与选择字段使用 10px 字段角色；反馈与下载进度使用 16px 反馈角色；网枪发射区使用 18px 面板角色；控制栏、来源栏和内容容器使用 24px 表面角色；模态确认使用 28px 对话框角色。媒体在大容器内裁出明确矩形边界；容器外缘保持圆角。工具栏、控制分组与健康栏常态无外围边框，表格保留必要的内部行分隔。状态圆点辅助真实状态。图标为内联线性 SVG，常规 18px、工作区与媒体工具 16px、低频文字按钮 15px，不用字符图标或装饰标题图标。

**The Rounded Controls Rule.** 小按钮两端半圆，大容器使用表面圆角；输入字段保持独立的较小圆角，不把所有对象改成同一轮廓。

## Components

### Buttons

按钮、输入与选择的最小高度统一 34px、无边框、两端半圆；石墨主按钮、浅灰次按钮、浅红底红字停止/删除按钮、透明低频按钮区分操作层级，同行按钮等宽。拦截链启停 36px，网枪发射 42px。低频工具、图像切换与反馈按钮可使用局部 28–32px 高度。不可逆确认用红色填充（button-danger-fill）并置于模态操作区；唯一的常驻例外是 fire_ready 时的发射按钮，以全宽实心红表示“现在可以单发”，条件未齐时退回浅红危险按钮或禁用态。禁用态使用更浅的独立填色与淡文字，与可用的浅灰次按钮明确区分，不恢复边框。focus-visible 为 2px 轮廓、2px 外偏移；背景与阴影使用 150ms ease-out。

下载按钮保持紧凑，最小高度 52px、内边距 9px 14px；统一使用同一线性下载符号，18px 图标居于 20px 列，间距 12px，文字列最大 148px 并可收缩。图标与文字组整体居中，三项图标和文字起点一致，文字组内左对齐，主行 13px/600、参数行 12px。800px 以下为三列，内边距缩为 9px 12px，保留下载图标；599px 以下恢复单列。

### Inputs / Fields

浅灰填色、无边框、10px 字段圆角与 frontmatter 的内边距。控制栏与检测预览的参数使用字段行：左侧 72px 辅助灰标签、右侧控件，行距 8px，避免“标签在上、控件在下”拉长控制栏。选择框统一 appearance:none 并绘制同一上下箭头。带单位的数值在字段内右侧显示单位（如估距等效尺寸的 cm），无障碍名称保留单位。placeholder 使用独立可读灰色且 opacity 为 1；焦点沿用公共轮廓。改名为原位置内联编辑，字段下方显示错误；失败保留重试入口与内容。

素材类型选择仅显示实际编码参数 LOSSLESS 与 H264_LOSSLESS，不在选项内追加常驻说明。YOLO 输入与实时预览不附常驻指导段落；保留操作状态、实际规格与必要的错误反馈。

### Navigation

工作区白色胶囊选中面、轻阴影和较强字重表示当前位置；未选中项透明，悬停为墨色文字。使用 tab、aria-selected 与方向键/Home/End。RGB / 检测叠加是浅灰胶囊内的图像局部分段切换，pressed 状态使用白色胶囊子按钮与轻阴影，不与硬件识别开关混淆。

### Status

状态由文字与 7px 圆点组成：正常运行为次级墨色文字加绿色圆点，注意为棕色文字加琥珀圆点，异常为红色文字加红色圆点，未运行或未知为辅助灰文字加灰色圆点。页头状态是 28px 白色胶囊（轻阴影），注意与异常时改为浅琥珀或浅红底；拦截任务标题旁的运行状态使用浅灰底胶囊。控制分组标题行右侧（如“YOLO 运行中”）只用文字加圆点，不加底色。顶部运行状态在相机与录像之后显示拦截链（未连接、读取失败、状态待更新、未运行、当前阶段或已发射）；599px 以下隐藏，由拦截任务栏承担。后台任务在页头以带旋转环的胶囊显示，空闲时隐藏。

### Cards / Containers

控制栏、来源栏、图像、目标表与回放使用白色、无外围边框、大圆角与共用表面阴影；保留各自功能尺度，不复制等尺寸卡片。标题区使用留白组织，标题与规格同一基线。实时画面下方为四个指标块（采集帧率、识别帧率、画面时效、图传码率）：12px 辅助灰标签在上、18px/600 数值与 12px 单位在下，块间 1px 内部分隔；599px 以下为 2×2。录像时长与可录时间用浅灰底的标签／数值两栏。

板上场次详情为定义列表：录制时间（附“北京时间”，整体换行不拆字）、大小、分段、状态与备注逐行显示，行间 1px 分隔；时间待核对与场次不完整以独立提示行显示。本机视频信息行中文不在词中断开（keep-all，必要时任意处断行），文件路径用等宽字体。

### Stable target table

固定外高、sticky 表头与复用行组成稳定测量组件，空表保留同一容器。表格按列拆分为 12 列：目标、置信度、定位、水平角、垂直角、X、Y、Z、轴向深度、距离、深度有效率、深度判定；单位写在表头（°、m）并以浅辅助灰小字显示，数值列右对齐，数值列与随后的文字列之间加宽间距。目标编号与距离为墨色 600，定位质量同时显示文字和颜色（有效深度绿、尺寸估计棕、仅方向灰）。不得用“/”把多项测量合并进一格；过期数据不能继续显示为有效结果。

### Mission panel

拦截任务栏是 24px 圆角白色表面，使用容器查询：容器宽度 ≥640px（两列与单列的宽窗口）时主体分两栏，≥860px 时就绪检查再分两列，窄容器为单栏。标题行为 16px/600 标题与浅灰底状态胶囊，其下 12px 辅助行显示“PID … · 日志 N 秒前更新”；启动与停止拦截链在同一行。

阶段轨道是 7 步有序列表（待命、起飞、跟踪、瞄准、开火、稳定、旋转）：已完成为绿色实心点，当前为石墨实心点加 3px 浅环、墨色 600 文字并带 aria-current="step"，未到为空心点与浅辅助灰文字，连接线 2px。开火步骤只在实际发射计数大于 0 时标为完成，不由阶段名推算。

发射区使用浅灰工具面与 18px 面板圆角：标题、发射模式、aria-live 状态文字（圆点与文字顶端对齐、可换行）、全宽 42px 发射按钮与单发闭锁说明。状态文字说明当前为何不能发射（未运行、自动模式、已闭锁或条件未齐）。

就绪检查是固定行的定义列表（感知输入、感知数据流、Offboard 心跳、飞控、解锁、控制模式、目标估计、目标距离、高度门、机体倾角、火控授权）：圆点固定在行首左列、标签辅助灰、数值右对齐并为墨色 500，行间 1px 分隔。good 只显示绿点，bad 红点加红字，warn 琥珀点加棕字，idle 为空心点加浅辅助灰文字；缺失数据显示为 —，不推算；目标估计无效时距离行标“估计无效”而不标窗口内，制导节点停发心跳时显示“已停发”，不长期停在“监测中”。行节点复用，数量不随状态变化。原始日志与操作要点折叠在 details 中，不常驻。

### Feedback / Confirmation

固定反馈位于设备健康栏上方，使用 16px 反馈圆角与独立阴影，错误用独立暗红面，文字可换行、可关闭。下载进度面使用同一反馈圆角。原生 dialog 使用 28px 对话框圆角与 24px 内边距，承载永久删除，明确板上范围与本机副本保留，初始焦点在取消。网枪发射使用同一 dialog 语法：标题声明单发，摘要显示实时就绪状态，说明不可撤回与板上火控复核；初始焦点在取消，条件失效时确认按钮同步禁用。不使用浏览器原生 confirm()。展开箭头只用 160ms ease-out 旋转；prefers-reduced-motion 禁用过渡。

### Device Health

CPU、GPU 使用率在健康栏靠前排列，沿用 12px 标签和 15px/600 对齐数字；所有窗口首次打开页面默认收起，摘要显示温度与两项使用率。首次 CPU 采样、读数缺失、过期或连接异常时显示未知，不把缺失写成 0%。模组供电保留 VDD_IN 的来源提示与非电池电压说明，不用它推算电量。

本机视频列表和播放器下方显示场次备注，内容来自本机保存的备注与场次信息；以纯文本呈现，可换行，空备注隐藏。备注不改变媒体身份、文件名或录制时间。

## Do's and Don'ts

### Do:

- Do 以真实 RGB、YOLO 画面和本机视频作为图像内容；图标采用现有内联 SVG。
- Do 保持固定目标容器、稳定滚动条、sticky 表头和滚动位置。
- Do 用文字配合状态色；未知、过期、未标定与错误保持明确。
- Do 正常状态用中性文字加绿点，只给注意与异常着色或加浅底。
- Do 控制参数使用“标签｜控件”字段行，单位放在字段内或表头。
- Do 目标表每项测量独立一列、数值右对齐，单位写在表头。
- Do 让桌面控制和素材来源栏 sticky 并内部滚动，窄屏恢复相应折叠或自然堆叠；反馈固定显示且不改变内容流。
- Do 保留键盘焦点、禁用、空状态、失败重试、取消与不可逆确认。
- Do 用胶囊按钮、分尺度圆角与轻环境阴影；常驻控件和表面保持无外围描边。
- Do 按 --media-room 等比缩小整个本机回放卡片，让 stage 占满卡片宽度，以原比例和 contain 展示完整画面。
- Do 将场次栏与视频卡片作为一个工作区整体居中，平衡两侧留白。
- Do 将 CPU/GPU 使用率作为只读健康信息，缺失、首次采样或过期时显示未知。
- Do 让网枪发射入口与实时画面同屏可见，并用状态文字说明不可发射的原因。
- Do 用同一 dialog 语法确认发射与删除等不可逆操作，初始焦点在取消，条件变化时同步禁用确认按钮。

### Don't:

- Don't 恢复蓝色主操作、全场 13px 正文或每个标题前的装饰图标。
- Don't 给中文标题加负字距，或用小于 12px 的文字承载信息。
- Don't 把每个正常状态都写成绿色文字，或用“/”把多项测量合并进一格。
- Don't 堆叠同尺寸白卡片、硬偏移阴影或装饰动画。
- Don't 把工作区切换与相机、识别、录像生命周期耦合。
- Don't 将历史通用检测配色、双源标记或灵敏度控件继承为日常组件。
- Don't 用模拟画面、装饰效果图或推算状态替代实际数据。
- Don't 将遗留边框变量继承为当前常驻控件边框，或把实时监看的 650px 上限施加到本机回放。
- Don't 用裁切或拉伸视频遮盖布局空白。
- Don't 恢复常驻 YOLO/预览说明或在编码参数选项内加入指导文字。
- Don't 把发射或拦截链控制放回需要滚动才能到达的控制栏底部，或用浏览器原生 confirm() 确认发射。
- Don't 在非 fire_ready 状态使用实心红按钮，或由阶段名推算已发射。
