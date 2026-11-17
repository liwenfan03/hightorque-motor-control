# Pipelining 设计方案（1 kHz，2-DOF）

状态：**已实现**（`include/hightorque/pipeline.hpp`、`src/pipeline.cpp`，2026-11-12），现在是 `motor_tool control` 所有 mode 的默认。
下面是最初的设计草案，保留作参考；实际行为和测试结果见 README 的"当前状态"和"Pipelining"各节。

## 1. 为什么需要

- 要求：控制 loop **固定 1 kHz**，不能降频。
- 现在的结构是顺序的：`发命令 → 等回复 → sleep → 下一个 tick`。`Fdcanusb::transact()` 会一直阻塞到收到回复（或超时）。
- 实测（`latency_probe`，`--rt`）：电机在工作模式里，回复的往返时间 p99 约 1.1 ms（mode 9）到 1.5 ms（mode 10），最大 2–4 ms。这条长尾来自电机 firmware，host 这边消不掉。
- 结果：顺序结构下，2–14% 的 tick 会迟到，loop 节拍被电机拖住。

## 2. 核心思路

**发和收分开，tick 永远不等回复。**

```
tick k:
  1. 非阻塞地读走所有已经到达的回复 → 更新每台电机的"最新状态"
  2. 把最新状态交给 controller，算出这一 tick 的命令
  3. 把所有电机的命令一次 write() 发出去 → 立刻返回
```

- 大多数 tick：上一 tick 的回复（约 0.6–0.9 ms 后到达）已经到了，状态的新旧程度和顺序结构**完全一样**。
- 回复迟到的 tick：用上一次的状态（旧 1 ms），**但 tick 本身准时**，命令也准时发出。
- 代价只在迟到的那些 tick：状态旧 1 ms（偶尔 2–3 ms）。

## 3. 接口设计

### 3.1 `Fdcanusb`（底层，新增两个非阻塞函数）

```cpp
// 把这些帧一次 write() 出去，不等 "OK"，不等回复。
void write_frames(const std::vector<CanFrame>& tx);
// 非阻塞：读走 USB 里已经到达的所有行，返回收到的 CAN 帧（"OK"/"ERR" 只计数）。
std::vector<CanFrame> poll_frames();
// 累计计数，用于诊断
uint64_t ok_count(), err_count();
```

现有的 `transact()` / `cycle()` **保持不变**，`test_motor`、`motor_tool` 继续用顺序结构。

### 3.2 `PipelinedBus`（新类，库里）

```cpp
struct MotorFeedback {
  MotorState state;                 // 最新一次收到的状态
  Clock::time_point received_at;    // 收到的时间（host 时钟）
  double round_trip_us;             // 这条回复对应的那次发送 → 收到
  uint32_t age_ticks;               // 已经连续多少个 tick 没有新状态（0 = 本 tick 新到的）
  uint32_t outstanding;             // 已发出、还没收到回复的请求数
  bool stale;                       // age_ticks > 上限（默认 5 ms）→ 应该报警 / 停机
};

class PipelinedBus {
 public:
  PipelinedBus(Fdcanusb& bus, std::vector<Motor*> motors);
  // 第 1 步：读走已到达的回复，返回每台电机的最新反馈（按 motors 顺序）。不阻塞。
  const std::vector<MotorFeedback>& update();
  // 第 3 步：发这一 tick 的命令（每台电机一个 FrameBuilder，例如 motor.command_vel(...)）。不阻塞。
  void send(const std::vector<FrameBuilder>& commands);
};
```

**`PipelinedBus` 自己不计时、不开 thread**。由调用者每个 tick 调一次 `update()` + `send()`：

- 现在在 Mac 上：由 `cli::Rate`（加 `--rt`）驱动。
- 以后在 Ubuntu 上：由 Franka / Flexiv SDK 的 1 kHz callback 驱动。这样 2-DOF 和 arm 用**同一个时钟**，不会出现两个 1 kHz loop 慢慢错开的问题。

### 3.3 回复怎么对应到请求

- 回复里没有序号，只有 source ID。按 ID 分到每台电机。
- 每台电机记一个"已发出请求"的时间队列：发一次 push，收到一条回复 pop 最老的一个 → 得到这条回复的 round trip。前提是同一台电机按顺序回复，CAN 是单一通道，可以认为成立，实现后用测试验证。
- `outstanding` 超过上限（例如 4）就说明电机不回复了：清空队列，标记 `stale`。

## 4. 安全

- **staleness 检查**：`age_ticks` 超过上限（默认 5，即 5 ms 没有新状态）就标记 `stale`。由调用者决定怎么处理：报警、停机或者保持。
- **电机端 watchdog**：开启电机自己的 timeout（register 0x01f，`motor_tool timeout <ms>`）。host 卡死或者 USB 被拔掉时，电机会自己刹车。pipelining 不再"每帧确认收到"，这一层保护更重要。**这个功能还没在真机上验证过**，实现时一起测。
- 退出时（Ctrl-C / 异常）照常用顺序结构发 `stop`，确认电机已经停下。

## 5. 带宽检查（2 台电机，1 kHz）

- 每 tick：2 个命令帧（float 约 20 byte）+ 2 个回复帧（约 24 byte）。
- CAN FD：仲裁段 1 Mbps，数据段 5 Mbps。每帧大约 60–80 µs，4 帧约 0.3 ms → **bus load 约 30%**，有余量。
- USB：每 tick 两行 `can send`，大约 120 byte，Full Speed 没有问题。
- int16 能再缩短一些。只有 2 台电机，float 也够用。

## 6. 和 9-DOF 整合的关系

- `PipelinedBus` 只负责 2-DOF 的收发，**不设 thread priority**。priority 由整合代码和 arm SDK 统一安排。
- 每个反馈都带 `received_at` 和 `age_ticks`，9-DOF controller 可以知道状态有多旧。需要的话，可以用 `pos + vel × age` 往前推。
- 同一条 CAN bus 只由一个 thread 使用（调用 `update()`/`send()` 的那个 thread）。

## 7. 测试计划

1. **unit test（不需要硬件）**：用假的帧流测试请求/回复的对应、`age_ticks`、`outstanding`、`stale` 的逻辑。
2. **新增 `pipeline_test` example**：N 台电机，固定 1 kHz，`--rt`。报告以下指标：
   - tick 准时率（tick 开始时间比预定时间晚多少：p50/p99/max）
   - 每台电机的 round trip（p50/p99/max）
   - 状态新旧分布：本 tick 新到 / 旧 1 tick / 旧 2+ tick 的比例
   - stale 次数、adapter 的 ERR 次数和 CAN 错误计数
3. 真机测试顺序（每一步都先确认）：
   - 只 query，电机不动；
   - vel = 0 锁住；
   - case 3 慢速转。
   每一步都是两台电机一起跑。

## 8. 不做的事

- 不改现有的 `transact()` / `cycle()` / `Motor::set_xxx()` 行为。
- 不开后台 thread（会带来锁和 priority inversion 的问题）。
- 不在 driver 里设 priority。

## 9. 待确认

- 4438 的具体型号：HTDW-4438-30 还是 -32？它决定力矩修正系数 k（`M4438_30` / `M4438_32`）。
- 4438 的 CAN ID：不能和 5047（ID 2）相同。
- stale 的上限（默认 5 ms）和触发后的动作。
