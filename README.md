# Robot Control UI

一个纯前端、手机友好的 iOS 风格机器人监控与控制网页原型。

## 已实现内容

- 顶部状态栏：机器人名称、连接状态、电量
- 实时视频区域：支持填写 HTTP 视频流地址
- 实时状态卡片：模式、目标、距离、行为、电量、连接状态
- 运动控制：Forward / Backward / Left / Right / Stop
- 模式切换：AUTO / MANUAL / AVOID
- 动作按钮：HELLO / PUSHUP
- 系统控制：Start Robot / Stop Robot / Shutdown（长按 2 秒触发）
- 事件日志
- Mock 模式：未接入真实机器人时可直接演示
- 无构建工具依赖，直接双击 `index.html` 即可打开

## 文件结构

```text
robot-control-ui/
├─ index.html   # 页面结构
├─ style.css    # iOS 风格样式
├─ app.js       # 交互逻辑、mock 数据、接口调用约定
└─ README.md    # 使用说明与未来接入说明
```

## 如何本地预览

### 方式一：直接打开

在文件管理器中打开：

- `robot-control-ui/index.html`

直接双击即可。

### 方式二：用本地静态服务器打开（更推荐）

如果你本机有 Python：

```bash
cd robot-control-ui
python -m http.server 8080
```

然后访问：

- `http://localhost:8080`

如果你本机有 Node.js：

```bash
npx serve .
```

## Mock 模式说明

页面右上角有 `Mock` 开关：

- **开启**：使用前端内置模拟状态，不依赖机器人设备
- **关闭**：尝试使用真实视频流和真实接口

Mock 模式下会：

- 自动轮播目标、距离、行为等状态
- 响应按钮操作
- 在日志中记录控制事件
- 支持演示连接/启动/停止/关机等状态变化

## 当前真实接口约定

当前原型里，前端预留了如下接口调用方式。

> 注意：这是**建议约定**，不是强制标准。你后端只要按这个结构适配即可。

### 1) 获取机器人状态

**GET** `/status`

返回 JSON 示例：

```json
{
  "robotName": "laton",
  "connected": true,
  "battery": 86,
  "mode": "AUTO",
  "target": "Charging Dock",
  "distance": "1.8m",
  "behavior": "Patrolling"
}
```

### 2) 运动控制

**POST** `/control/move`

请求体：

```json
{ "command": "forward" }
```

可选值：

- `forward`
- `backward`
- `left`
- `right`
- `stop`

### 3) 模式切换

**POST** `/control/mode`

请求体：

```json
{ "mode": "AUTO" }
```

可选值：

- `AUTO`
- `MANUAL`
- `AVOID`

### 4) 动作触发

**POST** `/control/action`

请求体：

```json
{ "action": "hello" }
```

可选值：

- `hello`
- `pushup`

### 5) 系统控制

**POST** `/system`

请求体：

```json
{ "system": "start" }
```

可选值：

- `start`
- `stop`
- `shutdown`

### 6) 视频流

前端当前支持输入一个 HTTP URL，例如：

```text
http://robot-ip:5000
```

原型默认将其直接作为图片流地址加载，适合：

- MJPEG 流
- 会持续刷新的 JPEG 快照接口
- 可被 `<img src="...">` 直接展示的流地址

如果你的机器人输出的是：

- HLS（`.m3u8`）
- MP4/WebRTC/WebSocket 视频

则建议后续把 `index.html` 中的视频区域改成：

- `<video>`
- WebRTC player
- canvas + WebSocket 解码方案

## 如何接入真实机器人

目前 `app.js` 中有一个 `state.apiBase` 字段，默认是空字符串。

### 最简单接法

如果网页与机器人 API 同域部署，例如：

- 页面：`http://robot-ip:8080`
- API：`http://robot-ip:8080/status`

那么可以保持：

```js
apiBase: ''
```

### 跨域部署接法

如果前端和后端不是同域，例如：

- 页面：`http://localhost:8080`
- API：`http://robot-ip:9000`

则可以在 `app.js` 里改成：

```js
apiBase: 'http://robot-ip:9000'
```

并确保后端启用 CORS。

## 建议的后续增强

- 增加 API Base 输入框
- 视频流类型切换（MJPEG / HLS / WebRTC）
- 长按运动控制，按住持续移动，松开自动 stop
- Joystick 虚拟摇杆
- WebSocket 实时状态推送
- 历史告警/任务面板
- 摄像头切换
- 深色模式切换
- PWA 安装支持

## 注意事项

- 这是 UI 原型，重点在展示与交互，不包含真实机器人安全保护逻辑
- 真机接入时，建议后端增加鉴权、限流、危险操作确认和命令去抖
- `Shutdown` 保留了长按机制，用于减少误触风险
