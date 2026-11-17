# 高擎电机 C++ host driver (Mac → mjcanfd-usb-1x → motor)

这是 vendor STM32 例程（`高擎电机fdcan协议解析v3.1.2/01-例程/fdcan_h730_v3.1.2`）的 C++ 移植版。
电脑直接当 host，不需要 STM32：

```
MacBook ──USB──> mjcanfd-usb-1x (CAN-FD adapter) ──CAN_H/CAN_L──> 高擎 motor <── 24 V power supply
```

CAN-FD frame 的字节内容和 vendor 的 `livelybot_fdcan.c` / `motor_many.c` 一模一样（`tests/protocol_test.cpp` 逐字节对比过）。
唯一的区别是 transport：原来是 `HAL_FDCAN_AddMessageToTxFifoQ()`，现在通过 USB serial 发 `can send ...` 文本命令给 adapter。

## 当前状态（2026-11-16，4438 + 5047 这一对）⭐ 先看这里

> 这一节是**现在的结论和推荐用法**。下面各节按时间顺序记录了调试过程，里面有些数字和"还没跑过 / 还没实现"的标注已经过时：遇到矛盾时以这一节为准。

### 硬件

| | ID 1 | ID 2 |
|---|---|---|
| 型号 | HTDW-4438-32（EE 关节，`M4438_32`） | HTDW-5047-36（`M5047_36`） |
| 软件限位（`src/zero.cpp`） | ±170° | ±90° |
| 固件 | 旧（没有 version register，**不支持 mode 21**） | 旧（同左） |
| 固件 kp / kd / ki（0x02b / 0x02c / 0x02d） | 1200 / 20 / 1 | 1000 / 30 / 1 |
| 静摩擦（起步） | 约 0.07–0.1 Nm | 约 0.25–0.3 Nm（动起来后 0.1–0.17 Nm） |

- 接线：MacBook → mjcanfd-usb-1x → 5047 → 4438（daisy chain），24 V 电源。
- CAN：5 Mbps 数据段（BRS），adapter sample point **0.75 / 0.75**（默认值，见"两台电机同一条 bus"）。换成更长的线以后，所有测试 CAN 都干净（err / warn / busoff = 0）。
- 零位存在电脑上 `~/.hightorque/zero_positions.txt`（不在 repo 里，每台机器自己 `set-zero`）。**不要用 `rezero` / `conf write`**：改 register 只改 RAM。
- 2026-11-16 ID 1 换了一台 4438（同型号、同旧固件、同 gains），零位重新设过。

### 推荐用法

```bash
cmake -S . -B build && cmake --build build -j && ./build/protocol_test
./build/motor_tool scan
./build/motor_tool go-zero --id 1,2                      # 两台同步回零
# 控制（配置文件 config/control.conf 给默认值；命令行优先）
./build/motor_tool control --mode impedance --id 1,2 --kp 20 --kd 0.3 --rate 800 --duration 20 --rt
./build/motor_tool control --mode position  --id 1,2 --profile sine --amp 20 --freq 0.25 --shape motor --rate 800 --rt
./build/motor_tool control --mode velocity  --id 1,2 --profile sine --amp 30 --freq 0.25 --rate 800 --rt
./build/motor_tool control --mode torque    --id 1,2 --profile square --amp 0.1 --rate 800 --rt
```

- **永远加 `--rt`**：不加时 macOS 会晚发约 2 ms，固定时间表的检查直接 FAIL。
- **控制频率：现在推荐 800 Hz**（见下表）。1 kHz 能跑，但有少量真丢包（固件问题，等新固件）。
- **position mode 用 `--shape motor`**（每帧带 vel-max / acc，由电机自己插值）。默认的 `--shape host` 在这台 4438 上会啸叫，原因见下面。
- 所有 mode 都是 pipelined（固定间隔发送，回复到了就用），结束时先刹车再 stop。

### 稳定性（2026-11-16，新 4438 + 长线，两台一起，`--rt`，每台丢失的回复）

| mode | 800 Hz，ID 1 / ID 2 | 1 kHz，ID 1 / ID 2 |
|---|---|---|
| position（`--shape motor`，32 B 帧） | 11 / 0 | 0.86% / 0.18% |
| velocity（24 B） | 0 / 0 | 0.16% / 0 |
| torque（24 B） | 3 / 0 | 0.10% / 0.017% |
| impedance（24 B） | 1 / 0（13 s） | 0.26% / 0.08% |

- 丢失全部在控制循环里，**停机阶段（刹车 + stop）每次都是 3/3 回复**。
- 丢的是**真丢**（回复永远不来），不是迟到；CAN 干净，所以不是 bus 的问题。

### 已经弄清楚的

1. **1 kHz 丢包 = 电机固件的 10 ms 任务。** 电机每 10 ms 有约 1–1.25 ms 不处理 CAN，这段时间里到达的请求被丢掉（不回复）。两次请求间隔 1.0 ms（1 kHz）时经常有一个落在里面，间隔 1.25 ms（800 Hz）时几乎不会。帧越长（32 B），停顿越长，丢得越多。host 端改不掉，等新固件。
2. **"迟到"和"丢失"不同。** 顺序结构（发 → 等 → 发，`--sequential`）只会看到迟到，最高约 300 Hz；pipelined 结构按精确计数（请求数 − 回复数，包括最后的 drain）判断真丢。
3. **position mode 的啸叫（这台 4438）。** `--shape host` 时电脑每个周期给一个新的位置目标，目标是阶梯形的。电机的位置环（推测是 D 项 / 速度环，即 derivative kick）把每一步变成一个短的力矩脉冲，脉冲频率 = 控制频率，所以听得到。
   - 证据：`--shape motor` 完全安静；`--ff-vel` 变小声；降低固件 kp（0x02b）没用；hold、velocity、torque、impedance、go-zero 都不叫；换线也一样叫。
   - 其它 mode 也是零阶保持，但命令没有被求导，每步只有约 0.003 Nm 的小台阶，听不到。impedance 的 kd 很大时也可能嗡嗡响，kd 要慢慢加。
   - 之前那台 4438 不叫（个体差异，原因没证实）。
4. **impedance（host 端）有效。** τ = kp(q_d − q) + kd(v_d − v)，限幅 `--max-tqe`（ID 1 0.3 Nm，ID 2 0.5 Nm），kp 单位 Nm/turn（20 ≈ 0.056 Nm/deg）。
   - 死区 ≈ 静摩擦 / kp；力矩顶到上限以后 kd 不起作用。5047 起步摩擦大，回零不如 4438 准。
   - 没有摩擦补偿和重力补偿（τ_ff）。

### 还没解决 / 待做

- 新 4438 的力矩读数偶尔跳到约 1.9 Nm（torque mode，电机不动），推测是测量异常，原因不明。
- 新 4438 velocity 波动比 5047 大（RMS 13 vs 6 deg/s）；go-zero 停住时会来回找位置（力矩 +0.29 / −0.14 Nm）。
- 电源设 24 V，两台时读数约 22.3 V（一台时 23.3 V）：要检查电源是不是进了 CC（限流），用万用表量。
- derivative kick 还没直接验证（把 0x02c 临时减半，看声音是否变小，测完改回 20）。
- 新固件到了以后：重测 1 kHz（加 `--trace`），重扫 sample point，试 mode 21（电机内部 PD）。
- 最终系统：这 2-DOF 加一个 7-DOF 机械臂 = 9-DOF，在 Ubuntu 上 1 kHz（`--rt` 的 thread priority 由整合代码统一安排）。

### Repo 里有什么

| 路径 | 内容 |
|---|---|
| `include/hightorque/`、`src/` | driver library：protocol、`Fdcanusb`（adapter）、`Motor`、零位、控制循环（`control`、`pipeline`） |
| `examples/` | `motor_tool`（主要工具）、`test_motor` / `test_motor_many`（vendor case）、`rate_test`、`latency_probe` |
| `config/control.conf` | `motor_tool control` 的默认参数 |
| `tests/protocol_test.cpp` | 离线单元测试（不需要硬件） |
| `tools/check_link.py` | 分析 `--log` / `--trace` 的丢包和迟到 |
| `experiments/velocity_trim.cpp` | 实验：velocity mode 的 host 端速度修正（不在 CMake 里，编译方法见文件开头） |
| `docs/pipelining_design.md` | pipelining 的设计说明 |

## Build

```bash
cd ~/Documents/research/motor_control
cmake -S . -B build && cmake --build build -j
./build/protocol_test        # offline test, no hardware needed
```

生成的程序都在 `build/`：

| Program | 对应 vendor 文件 | 用途 |
|---|---|---|
| `test_motor <case>` | `test/test_motor/test_motor.c` | 单电机，case 0–12 |
| `test_motor_many <case>` | `test/test_motor_many/test_motor_many.c` | 一拖多，case 0–9 |
| `rate_test` | `rate_test.py` | 测真实控制频率 (Hz) |
| `motor_tool` | `motor_config.c` 等 | scan / query / version / rezero / reset / timeout / read（读 register）/ adapter 命令 |

## 第一次运行（按顺序来）

1. 电机固定在桌上，轴上不要装东西。电源 24 V，current limit 先设低一点（大约 2.5 A）。
2. 插上 adapter，`./build/motor_tool list`，应该会看到 `/dev/cu.usbmodemXXXX`。
3. `./build/motor_tool scan`：找 ID 1–32 的电机，电机不会动。
4. `./build/motor_tool query --full`：读一次状态，也会显示 bus voltage 和 temperature。
5. `./build/test_motor 3`：速度模式 0.1 r/s，慢慢转。**Ctrl-C** 会先发 stop 再退出。

如果没有回复：检查电源、CAN 接线、termination（断电时 CAN_H–CAN_L 之间应该大约 60 Ω），以及 `--id`。
加 `--debug` 可以看到每一行发给 adapter / 从 adapter 收到的文本。

## 突然没有回复？先查 bus-off

```bash
./build/motor_tool status      # busoff=1 表示 adapter 已经下线，什么都发不出去
./build/motor_tool recover     # 重启 adapter 的 CAN controller
```

- adapter 出厂设置是 `can.autorecover 0`：一旦进入 bus-off 就不会自己恢复。现在所有程序启动时都会自动检查，并在需要时 restart。
- 每个程序都加 **`--no-brs`**：data phase 也跑 1 Mbps，速度慢一点，但很稳定（实测 0% 丢失）。

### 5 Mbps 的实验结果（2026-11-02，关掉 auto-retransmission，每组大约 5 秒的 query）

| adapter `can.fd_sample_point` | 0.45 | 0.50 | 0.55 | 0.60 | 0.666（默认） | 0.70 | ≥ 0.75 |
|---|---|---|---|---|---|---|---|
| 丢失率 | 1.7% | 1.9% | 23% | 43% | 58% | 84% | 100% |

- 默认设置下，5 Mbps 有一半以上的 frame 会出错。平时之所以看起来"几乎都正常"，是因为 auto-retransmission 把错误盖住了。等错误计数器攒满 → bus-off。
- 调 TDC offset 几乎没有影响；关掉 delay compensation 则完全不通。
- "sample point 越早越好"的规律说明问题出在 **timing**（时钟 / sample point 和电机不匹配）。不太像终端电阻或反射的问题（mentor 说终端是正确的，这和数据一致）。
- 参考：vendor 的 STM32 host（`Core/Src/fdcan.c`）用 24 MHz 晶振，nominal sample point 80%，data sample point 75%。adapter 默认两个都是 66.6%。

### ✅ 根本原因：nominal sample point 和电机不一致（2026-11-04 确认）

CAN FD 在 BRS bit 的 **nominal sample point** 切换到 5 Mbps。adapter 在 66.6% 切换，电机（和 vendor STM32 一样）在 80% 切换，数据段一开始双方就差了约半个快 bit。
trim 保持原值 64，关掉重发，每组约 5 秒：

| `can.sample_point` (nominal) | `can.fd_sample_point` (data) | 丢失率 |
|---|---|---|
| 0.666（默认） | 0.75 | 98% |
| 0.666（默认） | 0.666（默认） | 87% |
| 0.666（默认） | 0.50 | 0.5% |
| **0.80** | **0.75**（= vendor） | **0%** |
| 0.80 | 0.666 | 0% |
| 0.80 | 0.50 | 84% |

当时（只有 5047）的推荐设置是 0.80 / 0.75，和 vendor STM32 host 一致。

> **⚠️ 2026-11-11 起默认值改成 `0.75 / 0.75`**（`Fdcanusb::kMotorSamplePoint` / `kMotorDataSamplePoint`）。
> 原因：**4438 在 nominal 0.80 下会丢回复**，两台一起时两台都丢 20–75%。0.75 / 0.75 在两台的共同可用区间（nominal 约 0.725–0.785）中间，实测两台一起 0 丢包。详见下面"两台电机同一条 bus"和"CAN 丢包"。
>
> | nominal sample point | 5047 | 4438 |
> |---|---|---|
> | 0.700 | ❌ | ❌ |
> | 0.725 – 0.775 | ✅ | ✅ |
> | 0.790 | ✅ | ⚠️ |
> | 0.800（旧默认 / vendor） | ✅ | ❌ |
>
> 换新电机或新 firmware 时，要先用 `rate_test --sample-points N,D` 重新扫一遍，再决定要不要改默认值。

`clock.hsitrim` 保持 64。

**现在每个程序启动时都会自动设置这两个值**（`Fdcanusb::set_sample_points()`，只改 adapter 的 RAM，不写 flash，拔掉 USB 就恢复成 0.666 / 0.666）。
- 设置完会重启 adapter 的 CAN controller，所以也顺便清掉了 bus-off。
- 加 `--adapter-timing` 可以跳过这一步，保留 adapter 自己的值。
- `motor_tool adapter / status / recover` 不会改设置，`conf get` 读到的就是 adapter 当时的真实值。
- 自己写的 C++ 程序：打开 `Fdcanusb bus;` 以后调用一次 `bus.set_sample_points();`。
30 秒验证：float 47,985 frames、int16 53,231 frames，0 丢失，`err=0`；query-only 约 1650 Hz（float）/ 1840 Hz（int16）。
下面的 trim 实验也能降到 0%，但那是用时钟偏差去抵消 sample point 的偏差，而且 RC 振荡器会随温度漂移，所以不推荐。
- 所有实验只改了 RAM，最后都已经恢复成原值。

### adapter 时钟 trim（`clock.hsitrim`，原值 64）的实验结果

| trim | 64 | 61 | 60 | 59 | ≤ 58 |
|---|---|---|---|---|---|
| 5 Mbps 丢失率（sample point 0.666） | 62% | 33% | 20% | 12% | 完全不通（连 1 Mbps 也不通） |
| 1 Mbps 丢失率 | 0% | 0% | 0% | 0% | 完全不通 |

- 结果可以重复，而且变化方向一致：adapter 的 CAN 时钟来自可以 trim 的内部 RC 振荡器，**它相对电机偏快**，调慢以后，错误明显减少。
- 单独调 trim 或单独调 sample point，都降不到 0%；**两个一起调就可以**：trim 59–62 × fd_sample_point 0.45–0.50 的范围内，全部 0% 丢失。
- 中心点 **trim 61 + fd_sample_point 0.50**：在关掉重发的情况下跑 30 秒，54,684 个 frame，0 丢失，`err=0`。query-only 速度：float 约 1820 Hz，int16 约 1950 Hz。
- 还没完全解释的现象：trim ≤ 58 时，连 1 Mbps 都突然失效。单纯的频率偏差不太会造成这种断崖，所以除了频率以外，应该还有别的因素（timing margin）。另外 RC 振荡器会随温度漂移，所以选了这个范围的中间值。

已在真机上确认（2026-11-02）：电机 **ID 2**，24 V 电源读到 23.3 V。固件没有 version register（属于旧固件，vendor 代码会显示成 3.9.1）。
`--no-brs` + int16 下，单电机 query-only 的 `rate_test` 大约 1400 Hz，latency 大约 0.7 ms。

## `test_motor` 的 case 表（和 vendor `test_motor_control()` 同一编号）

| case | 模式 | vendor 函数 | 参数 (default) |
|---|---|---|---|
| 0 | DQ 电压 | `motor_set_dq_vlot` | `--volt 0.5` V |
| 1 | DQ 电流 | `motor_set_dq_current` | `--cur 0.2` A |
| 2 | 位置（最大速度；差距大时很猛） | `motor_set_pos` | `--pos 0.1` r |
| 3 | 速度 | `motor_set_vel` | `--vel 0.1` r/s |
| 4 | 力矩 | `motor_set_tqe` | `--tqe 0.1` Nm |
| 5 | 位置 + 速度 | `motor_set_pos_vel` | `--pos 0.1 --vel 0.1` |
| 6 | 位置 + 速度 + 最大力矩 | `motor_set_pos_vel_MAXtqe` | `--pos 0.2 --vel 0.1 --tqe nan` |
| 7 | 梯形控制 | `motor_set_pos_velmax_acc` | `--pos 0.1 --vel 0.5 --acc 0.1` |
| 8 | 运控模式2 (PD, mode 21) | `motor_set_pos_vel_tqe_kp_kd_2` | `--pos 0.5 --vel 0 --tqe 0 --kp 1 --kd 0.1`（⚠️ 实验室这台旧固件不支持：回复 write error `30 00 03`，电机留在 stop） |
| 9 | 刹车 | `motor_set_brake` | |
| 10 | 停止 | `motor_set_stop` | |
| 11 | 查询状态 | `motor_get_state_send` | |
| 12 | 固件版本 | `motor_get_version` | |

通用 options：`--id 1  --type float|int32|int16  --model M5047_36  --unit turns|rad|deg  --rate 100  --duration S  --port PATH  --full  --no-brs  --adapter-timing  --vendor-frames  --debug`

为了安全，有几个 default 比 vendor 的小：case 0 是 0.5 V（vendor 1.5 V），case 4 是 0.1 Nm（vendor 0.5 Nm），case 8 是 tqe 0、kd 0.1（vendor tqe 1 Nm、kd 1）。
需要的话用参数改回去就行，例如 `./build/test_motor 4 --tqe 0.5`。

### 真机测试结果（HTDW-5047-36-NE，ID 2，旧固件，2026-11-04）

| case | 结果 |
|---|---|
| 0 DQ 电压 | ✅ 0.5 V 推不动，1.0 V 勉强挪动，1.5 V 约 0.08 r/s |
| 1 DQ 电流 | ✅ 0.2 A 推不动，0.4 A 约 0.95 r/s |
| 2 位置 | ✅ 第一台：走 0.5 圈不到 1 秒（走 0.1 圈时力量太小推不动）；⚠️ 第二台：出厂状态下推不动（见"还没解释的"）|
| 3 速度 | ✅ 0.1 r/s，平均 0.100 r/s |
| 4 力矩 | ✅ 0.1 / 0.5 Nm 推不动，1.0 Nm 约 0.95 r/s |
| 5 / 6 / 7 | ✅ 位置误差 0.0001–0.003 圈 |
| 8 运控模式 2 | ❌ 旧固件没有 mode 21（第二台 5047 也一样：没有 version register，mode 21 被拒绝）|
| 9 / 10 / 11 / 12 | ✅ |

- 36:1 减速器的起步摩擦（breakaway torque）大约 0.5–1.0 Nm，所以很小的电压 / 电流 / 力矩只会出力，不会转。
- case 0 / 1 / 4 **不控制速度**：一旦推动就会一直加速，务必加 `--duration`。stop 是"放手"，电机还会滑一段；要立刻停用 case 9 brake。
- **需要停得很准，用 case 5 或 case 7（限速 / 带加减速）走到目标，不要用 case 2 全速冲。** 实测：case 5 以 0.1 r/s 到位，误差 0.0001 圈（0.04°）；case 2 全速走 0.5 圈，冲过头 0.0015 圈（约 0.5°），之后减速器的静摩擦让它停在那里，不会再拉回来。

## register 残留（sticky registers）⚠️

电机的 register 写进去以后**会一直保留**，直到被再次写入。stop、换 mode、甚至换一个程序都不会清掉它，只有电机重启（`motor_tool reset` 或断电）才会清。
而每个 vendor 函数只写它自己用到的 register，所以**前一个函数设的限制会影响后一个函数**。

真机上的对照实验（每组之前先 `motor_tool reset`，用 vendor 原样的 frame，2026-11-04）：

| 先跑 | 再跑 | 结果 |
|---|---|---|
| case 6（写了最大力矩 0x025、停止位置 0x026、速度 0x021） | case 2（只写目标位置 0x020） | ❌ case 2 差 0.5 圈也不动 |
| case 4（kp scale / kd scale 0x023 / 0x024 写成 0） | case 3 | ✅ 没有影响 |
| **case 8**（旧固件拒绝 mode 21） | case 3 | ❌ 不动，见下面"case 8 的 kp / kd 残留" |

vendor 代码有同样的问题。

**这里的处理（和 vendor 不一样，有意为之）**：mode 10 的每个命令，会把它**没用到**的可选 register 重置成"不起作用"的值：

| register | 含义 | 重置成 |
|---|---|---|
| 0x021 | 目标 / 前馈速度 | 0 |
| 0x022 | 前馈力矩 | 0 |
| 0x023 / 0x024 | kp scale / kd scale | 1 |
| 0x025 | 最大力矩 | NaN（不限制） |
| 0x026 | 停止位置 | NaN（没有） |
| 0x028 / 0x029 | 速度上限 / 加速度上限 | NaN（用电机默认值） |

- `Motor` 会记住哪些 register 现在可能被设过，只清这些。连续用同一个函数时，frame 和 vendor 的完全一样，不会变长。
- 新建的 `Motor`（也就是每个程序刚启动时）不知道之前的程序留下了什么，所以**第一条 mode 10 命令会把全部 8 个都清一次**（float 多约 40 字节，只有这一次）。`reset()` 之后则认为全部是干净的。
- 用 `cycle()` 时，命令要在发送前才 build（`command_xxx()` 会更新这个记录）。
- 想要和 vendor 一模一样的 frame：`opt.clear_sticky_registers = false;`（offline 的 vendor 字节对比测试就是这样做的）。
- 真机验证（第一台电机）：case 6 → case 2，修复前 case 2 不动，修复后不到 1 秒走完 0.5 圈。
- 一拖多（`MotorMany`）没有做这个处理。

### case 8 的 kp / kd 残留（旧固件）

旧固件会**拒绝** mode 21（回复 `30 00 03`），但同一个 frame 里写的 kp / kd（0x02b / 0x02c）**还是被存进去了**。而在旧固件上，这两个 register 也会影响 mode 10，所以跑过一次 case 8 以后，case 2 / 3 / 5 / 6 / 7 都会变软、推不动，直到 reset。

| register | 第二台 5047 出厂值 | 跑完 case 8（kp 1、kd 0.1）以后 |
|---|---|---|
| 0x02b | 0 | 1.245 |
| 0x02c | 30 | 0.1245 |
| 0x02d | 150 | 150 |

**处理**：`set_pos_vel_tqe_kp_kd()` 第一次调用前，先读出 0x02b / 0x02c 的原值；如果电机回复 mode 被拒绝，马上把原值写回去。真机验证：case 8 → 读 register 仍是 0 / 30 → case 3 正常转。
- 直接用 `command_pos_vel_tqe_kp_kd()` + `cycle()` 不会做这个检查。
- 如果位置类命令突然变软、推不动，先 `./build/motor_tool read 0x2b 2` 看一下（`motor_tool write <reg> <v>` 可以写，只改 RAM），`./build/motor_tool reset` 可以恢复出厂值（只改了 RAM，从来没有 `conf write`）。

### 还没解释的：第二台 5047 的位置控制很软

reset 以后、所有 register 都是出厂值的情况下，第二台电机的 case 2（只给目标位置）差 0.5 圈也完全不动（力矩只有 0.04 Nm）；case 5 / 6 / 7 能动，但都停在离目标约 0.03–0.04 圈的地方。第一台电机同样的命令都能准确到位。
可能和它的出厂 0x02b = 0 有关。验证（只写 RAM，`motor_tool write 0x2b <v>`，每次 case 2 走 0.5 圈、2 秒，做完 reset 恢复）：

| 0x02b | case 2 力矩 | 结果 |
|---|---|---|
| 0（出厂） | 约 0.01–0.04 Nm | 完全不动 |
| 1 | 0.33 → 0.41 Nm（慢慢变大） | 不动（还没超过静摩擦） |
| 3 | 约 0.2 Nm | 慢慢动了：2 秒走 0.08 圈 |

- **确认：0x02b 决定旧固件位置模式"拉回目标"的力。** 出厂 0 = 没有"弹簧"，所以 case 2 不动。
- 但它不是简单的 kp：力矩和 0x02b 不成正比（3 反而比 1 小），而且是慢慢涨起来的。可能是串级控制（位置环 → 速度环，0x02c / 0x02d 是速度环参数），这只是猜测，没有验证。
- 还没读过第一台电机的 0x02b。要把 0x02b 改成合适的值长期使用，需要先弄清楚它的单位，最好问 mentor / 高擎。

## 在自己的 C++ 程序里用

```cpp
#include "hightorque/motor.hpp"
using namespace hightorque;

Fdcanusb bus;                          // auto-detect /dev/cu.usbmodem*
bus.set_sample_points();               // motor timing 0.75 / 0.75 (adapter RAM only), needed for 5 Mbps
Motor::Options opt;
opt.model = MotorModel::M5047_36;      // HTDW-5047-36-NE, torque correction k (真实力矩 = k * tqe)
opt.type  = DataType::Int16;           // smaller frames = faster; position range only ±3.27 turns
Motor m1(bus, 1, opt), m2(bus, 2, opt);

m1.stop();                             // clear faults first
auto s = m1.set_pos_vel(0.25, 0.5);    // one call = send command + receive state
if (s) printf("%s\n", format_state(*s).c_str());

// Two motors in ONE round trip (frames written back to back, then both replies collected):
auto states = cycle(bus, {{&m1, m1.command_vel(0.1)}, {&m2, m2.command_vel(-0.1)}});

// 一拖多 mode: one CAN frame for all motors (int16 only, IDs must be 1..N)
MotorMany many(bus, 2);
many.vel(1, 0.1);
many.vel(2, 0.1);
auto st = many.send();                 // st[id - 1]
```

在 `CMakeLists.txt` 里链接 `hightorque` library 就可以。

## Code structure

```
include/hightorque/protocol.hpp   frame encode/decode, registers, modes, faults, units (no I/O)
include/hightorque/fdcanusb.hpp   USB serial transport for mjcanfd-usb-1x (POSIX termios + poll)
include/hightorque/motor.hpp      Motor (= motor_control.c) and MotorMany (= motor_many.c)
include/hightorque/zero.hpp       host-side zero, joint limits, go_to_zero / go_to_zero_many
include/hightorque/control.hpp    host control loop: position / velocity / torque / impedance, shared safety
src/*.cpp                         implementations
examples/*.cpp                    test_motor, test_motor_many, rate_test, motor_tool
experiments/velocity_trim.cpp     velocity + host speed correction (experiment, built by hand, see "Velocity mode")
tests/protocol_test.cpp           compares our frames with the vendor byte arrays and PDF examples
```

## 和 vendor C 代码不一样的地方（有意为之）

- **清除残留的 register**：mode 10 命令会重置它没用到的可选 register（详见上面"register 残留"）。vendor 不会，所以函数之间会互相影响。
- **case 8 被拒绝时恢复 kp / kd**：旧固件拒绝 mode 21 但仍存下 kp / kd，这里会写回原值（详见"case 8 的 kp / kd 残留"）。
- **电机拒绝的命令会显示出来**：reply 里的 write / read error（0x30 / 0x31）放进 `MotorState::errors`，`format_state()` 会打印出来；vendor 直接忽略。
- **Reply 是直接返回的**：vendor 在 `motor_process_state_all()` 里异步解析；这里每次 `set_xxx()` 都会等这台电机的回复，再返回 `std::optional<MotorState>`。
- **float → int 用 round**：vendor 是 truncate（例如 0.29 Nm × 100 = 28.99 会变成 28）。数值超出范围时 clamp 到 ±32767 / ±2147483647，所以不会意外变成表示 "no limit" 的 0x8000。
  （vendor 的 `data_limit()` 下限有 bug，会返回 +32768。）
- **`set_vel_acc` float 版**：vendor 的 position 写的是 `00 00 00 80`（float 里是 -0.0，不是 NaN，像是从 int32 版复制过来的）；这里写真正的 NaN。
- **固件版本解析**：vendor 的 minor 少了 `<< 4`（版本 ≥ 4.16 时会显示错），这里改正了。
- **一拖多 `0x17 0x01` 回复**：vendor 会把它当成 compact 格式来解析（解错）；这里按普通 reply 解析。
- 一拖多 frame 中间的填充字节和 vendor 一样是 0x00；普通 frame 的结尾补 0x50 (NOP)。

## 关于 1 kHz

- `./build/rate_test --type int16`：单电机，能跑多快就多快。
- `./build/rate_test --type int16 --ids 1,2`：两台电机 pipelined。
- `./build/rate_test --many --ids 1,2`：两台电机用一拖多，一个 frame 搞定。
- `./build/rate_test --type int16 --rate 1000`：固定 1 kHz，看 latency 和 missed。

真机结果（macOS，第一台 5047，int16，只 query，`--rate 1000`，30 秒）：28,967 个周期，0 丢帧；p50 623 µs，p99 873 µs，p99.9 2.4 ms，最大 20 ms；**0.63% 的周期超过 1 ms**。平均能到 1 kHz，但长尾（偶尔几 ms 的延迟）不满足硬实时。原因推测是 macOS 调度，还没验证。真正 deploy 的 Ubuntu 上要重新测。

**加 `--rt` 之后（real-time thread priority）长尾基本消失**。同样条件，各跑 30 秒：

| | 超过 1 ms | p50 | p99 | p99.9 | max |
|---|---|---|---|---|---|
| int16，不加 `--rt` | 444（1.53%） | 636 µs | 1125 µs | 2.6 ms | 11.5 ms |
| int16，`--rt` | 1（0.003%） | 604 µs | 772 µs | 824 µs | 1064 µs |
| int16，`--rt`（重复） | 0 | 571 µs | 772 µs | 828 µs | 916 µs |
| float，`--rt` | 0 | 642 µs | 806 µs | 882 µs | 984 µs |

`--rt` 在 macOS 上用 `THREAD_TIME_CONSTRAINT_POLICY`（CoreAudio 用的那种），不需要 root；在 Linux 上用 `SCHED_FIFO` 80 + `mlockall`（需要 root 或 rtprio 权限）。`rate_test` 和 `test_motor` 都支持。所以长尾的原因确实是 OS 调度，不是 USB / CAN。

**但是发运动命令时，`--rt` 也跑不稳 1 kHz**（float，`--rt`）：

| | 超过 1 ms | p50 | p99 | max |
|---|---|---|---|---|
| 只 query（电机不动） | 0 | 642 µs | 806 µs | 984 µs |
| `rate_test`，写 vel = 0（通电锁住，不转），10 秒 | 37.7% | 975 µs | 1714 µs | 3.9 ms |
| `test_motor 3 --vel 0.1`（真的在转），3 秒 | 17.0% | 872 µs | 1662 µs | 4.2 ms |

都是 0 丢帧。带写命令的帧只比 query 多约 12 byte（5 Mbps 下约 20 µs），解释不了多出来的几百 µs。`rate_test --hist` 的直方图否定了"错过 USB 1 ms 周期"的猜测：分布**不是两堆**，而是**整体右移约 300 µs**（峰值从 600–700 µs 移到 900–1000 µs），另外有一条拖到约 1.9 ms 的尾巴。也就是说每一帧带写命令的往返都固定多了约 300 µs，一来一回已经接近 1 ms，没有余量。

**拆分这 300 µs**：`./build/latency_probe --id 2 --rt`。它依次发 7 种帧，每种 1 kHz 跑 5 秒，相邻两种只差一个变量；电机最多收到 vel = 0。float 的结果（int16 趋势一样，每项少 20–60 µs）：

| 帧 | 电机状态 | bytes | >1 ms | p50 | p90 | p99 | max |
|---|---|---|---|---|---|---|---|
| 只 query | stop | 5 | 0.0% | 638 | 711 | 781 | 963 |
| NOP 填充 + query（长度同命令帧） | stop | 20 | 0.1% | 714 | 847 | 908 | 1040 |
| 写 mode 0 + query | stop | 8 | 0.0% | 680 | 746 | 814 | 962 |
| 只 query | mode 10，vel 0 | 5 | 5.4% | 749 | 860 | **1491** | 3560 |
| 只写 vel + query | mode 10 | 12 | 6.9% | 817 | 928 | 1529 | 1651 |
| 只写 mode 10 + query | mode 10 | 8 | 7.1% | 792 | 900 | 1499 | 1690 |
| mode + pos NaN + vel + query（现在的写法） | mode 10 | 20 | 8.6% | 907 | 989 | 1500 | 1697 |
| 只 query | mode 9，0 A（不出力） | 5 | 2.3% | 634 | 766 | 1079 | 2363 |
| mode 9 + 0 A + query（case 1 的帧） | mode 9 | 20 | 3.0% | 775 | 880 | 1181 | 3174 |

（单位 µs）结论：

- p50 多出的约 270 µs 大致由三部分组成：**电机在 mode 10 里本身就慢约 110 µs**，**帧变长**（5 → 20 byte）约 75 µs，**处理写命令**约 70–90 µs。每帧重写 mode 只占约 40 µs，不是主因。
- **长尾（p99 ≈ 1.5 ms）只要电机进入 mode 10 就出现，哪怕只 query**。stop 模式下不管帧多长、写不写，p99 都在 1 ms 以内。所以长尾来自**电机 firmware 这一侧**，推测是电机在跑控制环时 CAN 回复被延后（没有 firmware 源码，无法验证）。我们这边改帧格式消不掉它。
- mode 9（电流模式）也有长尾，但比 mode 10 轻（p99 约 1.1–1.2 ms vs 约 1.5 ms），说明**只要电机在工作模式里，回复就会偶尔变慢**，位置环比电流环更明显。mode 9 的两行是另一次运行测的，同一次运行的 mode 10 数字和上表相近。
- `>1 ms` 的百分比对 p50 很敏感（p50 已经接近 1 ms），所以不同次测量从 8% 到 37% 都有；看 p50 / p99 更可靠。
- 含义：**"发 → 等回复 → 再发" 的顺序结构在电机工作时跑不稳 1 kHz**。要稳定 1 kHz，需要 pipelining（不等回复就发下一帧，用上一周期的状态），或者降到 500 Hz。

> ⚠️ `--rt` 只是**测试工具**（`rate_test`、`test_motor`）的选项，代码在 `examples/cli_common.hpp`。**driver library（`Fdcanusb`、`Motor`）不设置任何 thread priority。** 以后在 Ubuntu 上和 Franka / Flexiv 整合成 9-DOF controller 时，priority 由整合代码统一安排（要配合机械臂 SDK 自己的 real-time thread），不要照搬这里写死的 `SCHED_FIFO` 80。Mac 上的 latency 数字只代表 2-DOF 的通信能力。`rate_test` 每秒会打印 p99 / p99.9 和超过一个周期的次数，结束时打印总计。

这些数字要在真实硬件上测。在 fake adapter（pseudo-terminal）上测出来的数没有参考意义，因为它没有 USB Full Speed 和 CAN 的延迟。
C++ 本身的开销很小，瓶颈会在 USB Full Speed（1 ms frame）和 adapter firmware（参考 mjbots 的 benchmark）。

## Safety

- 每个 example 一开始都会先发 `stop`（清除 fault），Ctrl-C 时也会先发 `stop` 再退出。
- 电机自己有 watchdog：`./build/motor_tool timeout 100` 表示 100 ms 没收到命令就刹车。
- fault 33（driver fault）或 40（undervoltage）通常表示电源的 current limit 太低。
- 位置模式 (case 2) 用最大速度和最大加速度运动，电流可能瞬间到 5–10 A。

## 两台电机同一条 bus（4438 + 5047）✅ 已解决：sample point 0.75 / 0.75

接线（daisy chain）：adapter → 5047 → 4438。ID 1 = HTDW-4438-32-NE（`--model M4438_32`，测试用的那台，0x02b/0x02c/0x02d = 1200/20/1，旧 firmware），ID 2 = 第一台 5047（1000/30/1）。

**1. 4438 的 nominal sample point 和 5047 不一样。** 用 `--sample-points NOMINAL,DATA`（只写 adapter RAM）逐个扫，每台单独 query，1 kHz，`--rt`：

| nominal | 0.700 | 0.725 | 0.750 | 0.775 | 0.790 | 0.800 | 0.875 |
|---|---|---|---|---|---|---|---|
| 5047（ID 2） | ❌ | ✅ | ✅ | ✅ | ✅ | ✅ | 未测 |
| 4438（ID 1） | ❌ | ✅ | ✅ | ✅ | ⚠️ | ❌ | ❌ |

4438 在 nominal 0.75 下，data 0.6–0.8 都稳定。

**复查：bus 上只接 4438**（adapter → 4438，没有 5047），1 kHz，`--rt`，只 query：nominal 0.700–0.790 都是 0 超时，0.800 出错（max 3.7 ms），0.825 / 0.875 完全不回复；data（nominal 0.75 时）0.60–0.80 都 0 超时，0.85 有 2 次超过 1 ms。0.75 / 0.75 跑 30 秒：29,017 个周期，0 超时，p50 576 µs，p99 759 µs，max 968 µs。所以 **4438 在 0.80 失败是它自己的特性**，和 5047 在不在 bus 上无关（只是有 5047 时 0.700 也不行）。

终端电阻：两台电机 CAN_H–CAN_L 都约 56.7 kΩ（没有内置 120 Ω），通电后也一样；整条 bus 只有 adapter 的 120 Ω（实测 115 Ω）。**4438 那一端缺终端**，要在接 4438 的插头上加 120 Ω。

**只接 4438、末端没有终端时的信号测试**（0.75 / 0.75，只 query，每次跑之前 CAN 重启、错误清零）：

| 测试 | 丢帧 | 超过 1 ms | p50 / p99 / max | 跑完后 adapter |
|---|---|---|---|---|
| 普通 query，1 kHz，60 秒（59,033 个周期） | 0 | 1（0.002%） | 629 / 823 / 1277 µs | lec=7 dlec=0 err=0 warn=0 |
| 64 byte 帧（`--pad 59`），1 kHz，30 秒 | 0 | 97.7% | 1160 / 1305 / 1970 µs | lec=7 dlec=0 err=0 warn=0 |
| 64 byte 帧，最快速度，30 秒（约 880 Hz） | 0 | 96.3% | 1138 / 1299 / 1408 µs | lec=7 dlec=0 err=0 warn=0 |

（lec / dlec = 7 表示"没有新的错误"。）即使是最长的 64 byte 帧、最高的 bus 负载，也**没有任何 CAN 错误、没有丢帧**；data sample point 0.60–0.80 都能用，窗口也很宽。所以**只接一台 4438、线短的时候，单端终端没有造成可测到的问题**。64 byte 帧超过 1 ms 是因为帧本身长（USB 上要传 128 个 hex 字符，CAN 数据段也更长），和 termination 无关。这个结果**不能**说明两台一起时的问题不是 termination：两台时 bus 更长，又多了一个节点。仍然建议按 CAN 规范在末端加 120 Ω。两台共同可用的区间约 0.725–0.785，**0.75 / 0.75 在中间**。（2026-11-11 起这就是默认值。）

**2.（2026-11-10 的记录，2026-11-11 复测结果不同，见下面"CAN 丢包"）两台一起跑时，5 Mbps 下 4438 还是丢回复**（0.75 / 0.75，只 query）：

| 测试 | ID 1 丢 | ID 2 丢 | adapter |
|---|---|---|---|
| 同时发，10 秒 | 394 | 316 | lec=2（form error），warn=1 |
| `--sequential`（一台一台问，回复不重叠） | 343 | 3 | lec=2，warn=1 |
| `--no-brs`（全 1 Mbps），同时发 | 0 | 0 | lec=0，干净 |
| `--no-brs --sequential` | 0 | 0 | lec=0 |

- 每台单独跑（另一台挂在 bus 上但不说话）都正常。只要 5047 也在 5 Mbps 下说话，4438 就丢。
- 1 Mbps 下全对，但两台一起的往返 p50 约 1.3 ms，到不了 1 kHz。
- 可能的原因（还没验证）：bus 的物理层（termination：如果每台电机都带 120 Ω，三个并联就是 40 Ω；接线 stub 长度），或者 4438 和 5047 在 5 Mbps 数据段的 bit timing 不兼容。先断电量 CAN_H–CAN_L 电阻（正常约 60 Ω）。
- **2026-11-11 复测**：同样 0.75 / 0.75、两台一起、末端仍然没有 120 Ω，却是 0 丢包（见下面"CAN 丢包"）。和 11-10 这张表为什么不同，还不知道（接线是否动过没有记录）。如果以后又出现丢包，先查接线和末端终端。

### CAN 丢包：验收标准（必须满足，不能靠重试）

- **2026-11-10**：5047 + 4438 在同一条 bus 上、数据段 5 Mbps、默认 sample point 0.80 / 0.75 时，4438 的回复丢 25–75%（`go-zero`：537 个周期里 179 个没回复）。
- **2026-11-11 复测（4438 末端仍然没有 120 Ω，只 query，`--rt`，两台同时发）**：
  | 设置 | 结果 |
  |---|---|
  | 5 Mbps，默认 0.80 / 0.75 | 100–1000 Hz 都丢 20–30%（两台都丢），adapter lec=1/2、warn=1 |
  | **5 Mbps，`--sample-points 0.75,0.75`** | **0 丢失、adapter 0 错误**：5 / 48 / 64 byte 帧 × 300 / 500 / 1000 Hz，每档 30 秒 |
  | 1 Mbps（`--no-brs`），0.80 | 48 byte 帧 300 / 400 Hz 0 丢失；64 byte 帧往返约 2.7 ms，超过 2.5 ms timeout 被算成"丢"（迟到，不是坏帧） |
  - 所以丢包的主因是 **nominal sample point 0.80 不适合 4438**（见上面"两台电机同一条 bus"的扫描表），不是缺终端。仍然建议在末端加 120 Ω。
  - 往返时间（stop 模式，两台）：5 Mbps 下 5 byte 约 0.7 ms、48 byte 约 1.15 ms、64 byte 约 1.4 ms；所以 1 kHz 只有短帧能跑满，长帧（float 的位置 / 力矩命令）在顺序结构下约 700–850 Hz。300 / 500 Hz 余量很大。
  - 还没测：电机在工作模式（mode 10 等）下的长尾，以及 60 秒验收。
- **健全的 CAN bus 不应该丢包**：CAN 在硬件层有 ACK 和自动重发，接线和终端电阻都正确时，丢失的回复应该是 0。参考：5047 单独接时，1 kHz 跑几万帧，丢失 0 次。
- **加了 120 Ω 之后的验收标准**：两台电机一起接，1 kHz（`--rt`），跑 60 秒：
  - 每台电机丢失的回复 = 0；
  - `motor_tool status` 的 `err` / `warn` / `busoff` = 0，`lec` / `dlec` 没有持续增加；
  - `go-zero` **不能出现** `WARNING: lost CAN replies`。
- 达不到就是硬件问题（终端电阻、接线、sample point、线长），要继续查，**不能用软件重试来掩盖**。
- 代码里现有的重试只是临时 workaround，而且**每次都会计数并打印出来**：
  - `go_to_zero()` 开始前读位置：最多重试 4 次（`start_query_retries`）。
  - 运动中丢回复：连续 20 次才放弃（`missed_replies` / `cycles`）。
  - 1 kHz controller / pipelining：不能依赖重试。丢回复的 tick 要标成 stale 并且计数（见 `docs/pipelining_design.md`）。

## case 4（力矩）在 1 kHz 下（5047，`--rt`，2 秒）

| 设置 | 每帧大小 | 实际周期数 / 2 s | 超过 1 ms | p50 / p99 / max | 电机 |
|---|---|---|---|---|---|
| float，0.5 Nm | 48 byte | 1673 | 98.8% | 1176 / 1842 / 3900 µs | 转到约 0.33 r/s（这次起步摩擦比之前小） |
| float，1.0 Nm | 48 byte | 1632 | 99.5% | 1212 / 1806 / 4110 µs | 约 0.95 r/s |
| int16，1.0 Nm | 24 byte | 1933 | 45.6% | 989 / 1726 / 3358 µs | 约 0.95 r/s |

- 都是 0 丢帧，力矩控制本身正常。但**顺序结构下 case 4 到不了 1 kHz**：case 4 每帧要写 6 个寄存器（pos NaN、vel 0、前馈力矩、kp scale 0、kd scale 0、max torque NaN），float 时帧被补到 48 byte。
- ⚠️ **int16 的位置只能表示 ±3.2767 圈**：纯力矩模式下电机一直转，读数会卡在 3.2767 r 不再变化（float 读到的实际位置是 4.1 r）。连续旋转时不能用 int16 读位置。
- 要真正稳定在 1 kHz：(1) 缩短帧：第一帧写完 kp/kd scale 等寄存器后，之后每帧只更新前馈力矩；(2) pipelining（见 `docs/pipelining_design.md`），因为 mode 10 本身就有 p99 约 1.5 ms 的长尾。两项都要改 driver，还没做。

**安全修正**：之前 `test_motor` 的输出接到 `| head` 或 `| grep -m` 时，管道一关程序就被 SIGPIPE 杀掉，来不及发 stop，电机会继续转（实际发生过一次，1.0 Nm 下多转了约 1 秒）。现在 examples 忽略 SIGPIPE，SIGHUP 也会正常停机。

## 零位（host 端软件零位）

```bash
./build/motor_tool set-zero --id 2      # 先让电机 stop，用手掰到想要的位置，再运行：记下当前位置
./build/motor_tool zero --id 2          # 看存的零位，以及现在离零位多远（不动）
./build/motor_tool go-zero --id 2       # 回到最近一次 set-zero 的位置（限速 0.2 r/s、加速度 0.5 r/s²，case 7）
./build/motor_tool zero --id 1,2 --model M4438_32,M5047_36      # 两台一起看
./build/motor_tool go-zero --id 1,2 --model M4438_32,M5047_36   # 两台同时回零（同步）
```

- **两台同时回零**（`--id 1,2`，库函数 `go_to_zero_many()`）：
  - host 端统一规划一条 minimum-jerk 时间曲线 s(t)（0 → 1，两端速度、加速度都是 0）。每个周期每台电机的目标是 `起点 + s(t) × (零位 − 起点)`，所以**同时出发、每一刻都走到各自路程的同一比例、同时到达**。时长 T 取"每台电机都不超过 `--vel` / `--acc`"的最短时间，由离得最远的那台决定。
  - 所有电机的命令放在同一次 `cycle()` 里（一次 USB 写入，CAN 上两帧相差约 0.1 ms）。
  - T 之后每台各自做摩擦修正；**全部进入容差（停稳 0.3 s）才算到位**；超时是 T + `--timeout`。
  - 任何一台出限位、丢回复太多、超时或者 Ctrl-C，**所有电机都 stop**（发 stop，并确认 mode 0，最多试 3 次）。
  - 开始前每台都检查零位、限位、当前位置；有一台不行就都不动。
  - `--model` 可以给一个（所有电机共用），也可以按 `--id` 的顺序每台给一个。
  - 单台的 `--id 2` 还是原来的方式（用电机自己的梯形规划），两种方式都测过。

- 零位存在电脑上的 `~/.hightorque/zero_positions.txt`（每个 ID 一行，`--zero-file PATH` 可以换文件），**不写电机 flash**。vendor 的 `motor_tool rezero` 会 `conf write`，把电机 RAM 里的全部设置写进 flash，不建议用。
- 库函数：`include/hightorque/zero.hpp` 里的 `set_zero_here()` / `go_to_zero()` / `load_zero()`。
- **电机在限位外时**（被手转过、或被推出去）：`go-zero` 不会动，而是打印哪台在哪、限位是多少、往哪个方向至少转多少度，把它设成 stop（可以用手转），然后等你按 Enter 再检查一次，直到都回到限位内才开始回零（输入 `q` + Enter 取消，什么都不动）。不是在 terminal 里运行时（例如被脚本调用）只打印问题然后退出。`--force` 跳过这个检查。
- `go-zero` 选项：`--vel` `--acc` `--tol`（容差，默认 0.002 r）`--timeout`（默认 15 s）`--hold`（到位后保持，Ctrl-C 松开）`--force`（跳过限位检查）。
- **断电行为（2026-11-09/10，5047 #1 实测）**：电机断电后**保留一圈以内的角度，丢掉整圈数**。上电后读数被折进一个固定的一圈窗口：窗口里的位置断电前后读数不变，窗口外的会差整数圈。
  - 断电前后不变的点：−0.26086、−0.17920、+0.07153（断电一整晚）、+0.51062 r；−2.67725 上电后变成 +0.32292。
  - 由这些点推出的窗口：[−0.260, +0.510] r 一定在里面（真正的边界：下边界在 −0.489 到 −0.261 之间，上边界在 +0.511 到 +0.739 之间）。窗口不以 0 为中心，推测和 flash 里的零点偏移有关（没验证）。**代码已经不依赖这个窗口了**，见下一条。
  - 方向：从输出端看，**逆时针为正**。
- **相对零位的角度 = `wrap(读数 − 零位)`，折回到 ±180° 以内**（`wrap_turns()` / `effective_zero()`）。读数只会差整数圈，所以不管窗口在哪里、读数里带了几圈，算出来的都是关节的真实角度。开机时按当前读数算一次"有效零位"（零位 + 整数圈），之后所有目标都是 有效零位 + 角度。
  - 零位可以设在任何地方；换电机也不用测窗口；通电时转过整圈也没关系。
  - 前提：关节离零位永远不会超过半圈，所以限位必须**严格小于 ±180°**。
  - 检测不出来的情况：断电时被转了接近整整一圈。
- **软件限位（写死在 `src/zero.cpp` 的 `kJointLimits`，按 ID / 关节）**：
  - ID 2（5047 关节）：±90°，是 5047 法兰的转动范围。履带传动比 1:1，所以关节也是 ±90°。
  - ID 1（4438 EE 关节）：**±170°**。这个关节上没有线缆（4438 外壳跟着 5047 那一侧动，法兰带着 EE 转），本来可以无限转，但为了让 9-DOF controller 有一个有界的关节范围，特意限制在 ±170°。180° 两边各 10° 是"不允许停留"的区域：在 180° 附近，读数分不清关节是从哪一边过来的；另外这段区域也留给过冲和"断电时被掰出限位"的检测。在 +160° 和 −160° 之间移动，会走远的那一边（320°），不会穿过 180°。
  - 代码也支持连续关节（`continuous = true`：不设限位，回零走近的那一边，最多 180°），改 `kJointLimits` 的一行就能切换。连续关节必须用 float（或 int32）数据类型：通电期间原始读数会一直累计圈数，而 int16 位置在 ±3.2767 r 就会饱和。`go-zero` 遇到"连续关节 + int16"会直接拒绝。
  - 4438 测试机，2026-11-10：零位 1.2258 r（读数比开机时多一整圈，代码会自动忽略）。从 −176° 回零（当时还是连续关节，走了近的那一边），速度 0.1 r/s，用时 5.6 s，最终误差 −0.35°，没用到修正。运动时速度在 0.06–0.13 r/s 之间跳，力矩在 0.12–0.26 Nm 之间波动，可能和减速器有关（只测了一次）。另外读 20 次只收到 10–11 次回复（4438 那一端没有终端电阻）。
- **安全检查**：
  - `go-zero`：没有限位、或者当前位置在限位外（±3.6° 余量），都**不发任何命令就拒绝**；移动过程中跑出限位就 stop。在限位外时：电机 stop 的状态下，用手按线缆允许的方向转回限位里，再试。
  - `zero`：显示离零位多少度、限位，以及"现在是否在限位里"；读数带着整数圈时也会提示。
  - `--force`：跳过限位检查，只用于恢复（仍然最多转半圈，走近的那一边）。
  - 没到位（超时 / Ctrl-C / 没回复 / 出限位）都会自动 stop。
  - 以后的 controller：每个位置目标都要先过 `clamp_to_limits()`，并且使用有效零位。
- **回零精度（2026-11-10，5047 #1）**：电机自己的位置环会在离目标约 −0.007 r（−2.5°）的地方被**静摩擦**卡住。卡住时电机一直在往目标用力（+0.2–0.4 Nm），但轴不动。从正方向回来时会先冲过零位（+1.2° 时速度还有 −0.1 r/s），然后在 −2.1° 卡住，所以两个方向都停在负的一侧。
  - `go-zero` 加了 **host 端慢速积分修正**：电机几乎静止、离零位不到 18° 且误差大于半个容差时，把发给电机的目标按 `−0.5/s × 误差` 慢慢推，最多 ±0.03 r，并且始终限制在限位以内（`--corr-rate`、`--corr-max`，设 0 可以关掉）。`--hold` 会保持在最后的命令位置（零位 + 修正）。
  - 实测结果：

    | 起点 | 用时 | 最终误差 | 修正量 |
    |---|---|---|---|
    | −2.5° | 2.8 s | −0.70° | +0.0050 r |
    | −57.7° | 4.3 s | −0.53° | +0.0056 r |
    | −60.7° | 4.0 s | −0.70° | +0.0054 r |
    | +57.6° | 4.0 s | −0.70° | +0.0053 r |

  - 全部在 0.72° 的容差以内，但经常贴着容差的边：误差一进入容差、停稳 0.3 s 程序就结束了。
  - 修正之前（2026-11-05/10）：误差会停在 2.4–2.6°，15 s 后超时。

## Host 控制循环（`motor_tool control`）：position / velocity / torque / impedance

四种控制用**同一个 host 循环**（`run_control()`，`include/hightorque/control.hpp`）：每个周期（默认 300 Hz）由轨迹函数给每台电机一个 setpoint，转成一条命令，所有电机在一次 `cycle()` 里发出。角度都相对存好的零位（和 go-zero 一样按 ±180° 折叠）。

| mode | 发给电机的 | setpoint | 说明 |
|---|---|---|---|
| `position` | 位置模式（mode 10），寄存器按参数发（见下表） | 位置（deg） | 目标先 clamp 到软件限位。只给 `--vel-max` + `--acc` = case 7，什么都不给 = case 2 |
| `velocity` | 位置模式，位置 = NaN，速度 = setpoint，其它寄存器按参数发 | 速度（deg/s） | host 按 `--vel-ramp`（默认 1 r/s²）限制速度变化；靠近限位时按 v ≤ √(2·a·d) 减速，在限位内 3.6° 停下 |
| `torque` | case 4（`tqe`） | 力矩（Nm） | τ = setpoint − kd·v（`--kd` 可选阻尼），限幅 `--max-tqe`（默认 0.3 Nm） |
| `impedance` | case 4 | 位置（deg） | τ = kp(q_d − q) + kd(v_d − v)，限幅 `--max-tqe`；kp / kd 在 `--gain-ramp`（1 s）内从 0 升上来 |

**位置模式寄存器**（position / velocity mode；不给 = 不发，电机用固件默认；之前命令留下的值会被清掉）。所有位置类的 vendor case 都只是它们的组合（库函数 `Motor::command_position_mode()`，单元测试确认和 case 2 / 3 / 6 / 7 的帧逐字节相同）：

| 参数 | 寄存器 | 单位 | 不给时 |
|---|---|---|---|
| `--vel`（position 时加 `--ff-vel` = 发 setpoint 的速度） | 0x021 速度 / 前馈速度 | r/s | 0 |
| `--tqe` | 0x022 前馈力矩 | Nm | 0 |
| `--kp-scale` / `--kd-scale` | 0x023 / 0x024 固件 kp / kd 的比例 | 0–1 | 1 |
| `--max-tqe` | 0x025 最大力矩 | Nm | 不限 |
| `--stop-pos` | 0x026 停止位置（相对零位） | deg | 无 |
| `--vel-max` | 0x028 速度上限 | r/s | 固件默认 |
| `--acc` | 0x029 加速度上限 | r/s² | 固件默认 |

**不变的寄存器只发一次**（`--resend-every N`，默认 100，配置文件里也有）：position / velocity mode 下，和上一个周期相同的寄存器不再发（电机会一直保持它们），位置每个周期都发；每 N 个周期（和第一个周期）完整重发一次，防止电机重启后丢了它们。帧长：位置 + max-tqe + vel-max + acc 每周期都发 = 32 byte，只发位置 = 16 byte（float）。`--resend-every 1` = 以前的行为（每个周期全发）。库函数 `Motor::command_position_mode_changed()`；中间有别的命令（例如 torque）或 `reset()` 时缓存失效，下一帧自动完整发送。单元测试覆盖。实测（position 500 Hz，20 秒，`--trace`，数据类型都是 float32）：

| | 每周期全发（32 byte） | 只发变化的（16 byte） |
|---|---|---|
| 单台延迟 p50 ID 1 / ID 2 | 1061 / 1195 µs | **845 / 921 µs** |
| 单台延迟 p99 ID 1 / ID 2 | 1843 / 1836 µs | **1563 / 1713 µs** |
| 一个周期往返 p50 / p99 | 1240 / 1946 µs | **974 / 1769 µs** |
| 没按时收到 ID 1 / ID 2 | 7 / 5 | **0 / 0** |
| overruns | 74 | **2** |
| 位置误差 RMS ID 1 / ID 2 | 1.58° / 1.38° | 1.33° / 1.27° |

→ 基础往返快了约 20%，500 Hz 这次 0 迟到。10 ms 的固件长尾还在（p99 约 1.6–1.7 ms）。1 kHz 仍然需要 pipelining（p50 已接近 1 ms，p99 远超）。

> ⚠️ **2026-11-12 发现：默认已改回 `resend-every = 1`（每个周期全发）。** pipelined 1 kHz 方波（±15°，vel-max 0.3 r/s，acc 1 r/s²，resend-every 100）在第一次跳变（第 2001 帧，只发了位置）几毫秒内速度就超过 0.5 r/s，`safe_vel` 停机。如果 vel-max / acc 还在起作用，30° 至少要约 0.3 s。推断：**速度上限 0x028 / 加速度上限 0x029 只在带着它们的那一帧里起作用**，不像 0x023 / 0x024 那样保留（之前是假设，没验证）。所以上面 32 → 16 byte 的"改善"（包括跟踪误差变小）很可能是因为限制没起作用。正弦的速度很小，碰不到限制，所以之前没发现。go-zero、torque、impedance 每帧都全发，不受影响。
>
> **验证（同一天，pipelined 1 kHz，方波 ±10°，`--resend-every 1`）**：每一步都按规划走完（20° 约 0.47 s，和理论一致），没有超速，最后停在离目标 0.06–0.62°（摩擦）。→ 0x028 / 0x029 必须每帧都发。代价：帧 32 byte 时 1 kHz pipelined 的丢失变成 ID 1 0.60%、ID 2 0.72%（16 byte 时 0.14–0.18% / 0.05–0.07%），回复延迟 p50 993 / 1244 µs（ID 2 已超过 1 ms 周期，age 大多 2）→ 按判定标准 FAIL。

**控制频率**：`--rate HZ`（默认 300）。回复 timeout 自动设为约 0.9 个周期（最少 1 ms）。按"不能丢包"的标准，现在的顺序结构最高约 300 Hz（见上面"控制频率"）；更高的 rate 可以跑，link 报告会显示 late。

安全：`--safe-vel`（默认 0.5 r/s，任何一台超过就全停；**0 = 关掉**，会打印警告；配置文件里 `[position]` 放宽到 2.0 r/s，因为 position mode 的速度由电机的 vel-max / acc 限制）、`--max-start-deg`（默认 10°）。关掉速度检查时，限位、fault、丢回复的检查照样有效，Ctrl-C 照样停。

setpoint：`--profile hold|sine|square`、`--bias`（中心值）、`--amp`（幅度）、`--freq`（Hz，默认 0.5）、`--ramp`（开始 / 结束时幅度在 1 s 内渐入渐出）。单位跟着 mode：position / impedance 是 deg，velocity 是 deg/s，torque 是 Nm。每个值可以一个（所有电机）或每个 `--id` 一个。

**安全检查（所有 mode 一样，任何一台出问题 → 两台一起 stop 并确认 mode 0）**：
- 开始前（不发任何命令）：零位、限位、回复、在限位内、setpoint 在限位内、position / impedance 时离开始的 setpoint 不超过 `--max-start-deg`（10°）、参数有效、ID 不重复。
- 每个周期：超出限位、`|速度| > --vel-limit`（默认 0.5 r/s）、fault、连续 3 个周期没回复。
- 结束时打印被控量的跟踪误差（position：deg，velocity：deg/s，torque：Nm）的 RMS / max，以及每台丢失的回复数（必须是 0）。`--log file.csv` 记录每个周期。
- ⚠️ 进程被 kill（不是 Ctrl-C）时，velocity / torque mode 会保持最后的命令。

```bash
# position：±20° 正弦，0.25 Hz，300 Hz loop，速度 / 加速度上限 0.3 r/s / 1 r/s²
./build/motor_tool control --mode position --id 1,2 --model M4438_32,M5047_36 --rate 300 --profile sine --amp 20 --freq 0.25 --vel-max 0.3 --acc 1 --duration 20 --rt
# velocity：±30 deg/s 正弦
./build/motor_tool control --mode velocity --id 1,2 --model M4438_32,M5047_36 --rate 300 --profile sine --amp 30 --freq 0.25 --vel-ramp 1 --duration 20 --rt
# torque：±0.1 Nm 方波，带一点阻尼
./build/motor_tool control --mode torque --id 1,2 --model M4438_32,M5047_36 --rate 300 --profile square --amp 0.1 --freq 0.5 --kd 0.2 --max-tqe 0.3 --duration 10 --rt
```

⚠️ 2026-11-12：代码和单元测试完成，position / velocity / torque 三个 mode **还没在电机上跑过**。

### Pipelining（`--pipeline`，position mode）⚠️ 2026-11-12 代码完成，还没在电机上跑过

`run_control()`（默认）是"发 → **等**所有回复（或超时）→ 下一周期"：回复晚了，下一次发送也跟着晚，频率最高 ≈ 1 / 往返时间。`--pipeline` 用另一个函数 `run_position_pipelined()`（`include/hightorque/pipeline.hpp`，`run_control()` 没有改）：

- **固定间隔发送**：每个周期按时间表准时写出所有电机的命令，不管回复有没有到。
- **回复到了就用**：等下一个发送时刻的同时收回复。每台电机按顺序回复、不丢（见下面的实验），所以第 k 个回复 = 第 k 个请求；每个状态都知道自己"晚了几个周期"（age：1 = 回答上一个周期的请求）。
- 命令用每台电机最新的状态（位置模式下状态只用于安全检查和记录；位置环在电机里）。
- 安全：和 `run_control()` 一样的开始前检查、限位、速度、fault；另外**某台连续 `--max-stale`（默认 10）个周期没有新回复**、或**未回复的请求超过 `--max-outstanding`（16）**、或 adapter 报 ERR → 两台一起停。结束时 stop、等 100 ms、精确计数。
- 报告：发送有没有按时（晚于时间表 10% 周期以上的次数、最晚多少）、每台 请求 / 回复 / 丢失、回复延迟 p50 / p99 / max、**每个回复被用到时的 age 分布**、没有新回复的周期数和最长连续次数、adapter 状态。

```bash
./build/motor_tool control --mode position --id 1,2 --profile sine --amp 20 --freq 0.25 --rate 1000 --duration 20 --rt --pipeline --trace /tmp/pipe1000.txt
```

- **第一版的 bug（2026-11-12，已修）**：等待回复时，adapter 的 `OK` 行（每个 `can send` 一行）让等待提前结束，循环以约 3 kHz 发送（请求间隔 60–900 µs），两台电机大部分请求都没有回复（17 个里 10–13 个），10 ms 内触发 LostReplies 两台一起停（安全检查正常）。修复：`wait_frames()` 一直等到真正的 CAN 帧或截止时间；发送永远不早于时间表。
- **新发现**：请求间隔约 0.1–0.3 ms 时电机会丢回复（之前 0.5 ms 间隔 0 丢失），所以同一台电机的命令不能太密。1 kHz（1 ms）没问题。
- **mode 10 实测（position 正弦，2026-11-12，修复 OK 行之后）**：
  - 发送时间表完全守住：800 Hz 16000 周期 / 20.00 s（只 1 次晚 127 µs），1000 Hz 10528 周期 / 10.53 s（最晚 70 µs）。
  - **但有少量请求没有回复**：800 Hz 每台 2 个，1000 Hz ID 1 17 个、ID 2 5 个。trace 定位：(1) **开头**的 #1 / #3 / #4（第一帧切 mode 10 + 清寄存器要约 1.8 ms，其间到的请求被丢）；(2) 只在 1000 Hz，ID 1 在 6.8275 / 6.8375 / 6.8475 … s 各丢一个，**正好 10 ms 一个** = 和固件 10 ms 任务撞上：电机停顿 > 1 ms 时，下一个 1 ms 后到的请求被丢（电机的接收缓存很小）。ID 1 的停顿成串更频繁（每约 0.5 s），所以丢得比 ID 2 多。800 Hz（间隔 1.25 ms）开头之后 20 s 一个都没丢。
  - 报告里的延迟 3.3 ms（800 Hz）/ 最多 16.9 ms（1000 Hz）**是假的**：按顺序配对时，丢一个就整体错位一个周期。1000 Hz 时 ID 1 丢到第 17 个，"未回复请求 > 16" 触发停机（误判）。用 trace 离线重新对齐后真实延迟 p50：800 Hz 827 / 924 µs，1000 Hz 802 / 889 µs（和顺序结构一样）。
  - 修复：(a) 开始固定间隔之前先做 `--warmup`（默认 3）个"发 → 等回复"周期；(b) **丢失后自动重新对齐**：收到回复后，如果"发出超过 400 µs 还没回复的请求"连续 20 ms 都不为 0，就把最旧的算作丢失并划掉（离线重放：800 Hz 找到 2 / 2、2 / 2，1000 Hz 13 / 17、5 / 5，剩下 4 个是结束前不到 20 ms 的）。报告多一项 "found while running"。
- **加上 warm-up 和重新对齐之后（position 正弦，20 秒）**：
  | | 发一收一 1 kHz | pipelined 800 Hz | pipelined 1 kHz |
  |---|---|---|---|
  | 实际频率 | 922 Hz | **800 Hz** | **1000 Hz** |
  | 按时发送 | 27% 周期超时 | 全部（最晚 112 µs） | 1 次晚 107 µs |
  | 丢失 ID 1 / ID 2 | 0 / 0 | **0 / 0** | 32 / 4（全部发现并重新对齐） |
  | 回复延迟 p50 / max | 825–908 µs / 4.0–4.4 ms | 825–894 µs / **1.26–1.31 ms** | 861–904 µs / 7.5、2.9 ms |
  | 状态 age | — | 1: 94–96%，2: 4–6% | 1: 81–85%，2: 13–19%，ID 1 偶尔 3–8 |
  | 位置误差 RMS | 1.32° / 1.26° | 1.33° / 1.26° | 1.33° / 1.26° |
  - **800 Hz pipelined 通过验收**（0 丢失、按时、无 CAN 错误）。
  - 1 kHz：ID 1（4438）约 0.16% 的请求没有回复，ID 2 约 0.02%：固件 10 ms 停顿 > 1 ms 时丢下一个请求。还没解决（候选：检测到停顿时给那台电机跳过一次发送）。
  - 位置跟踪和频率无关（位置环在电机里）。
- 修复后只 query 的 pipelined 测试（stop 模式，5 秒）：1000 Hz 发送间隔 p50 1000 µs（925–1079），800 Hz p50 1250 µs；两台 5000 / 5000、4000 / 4000 个回复，0 丢失，全部 age 1。mode 10 下还没测。

### 停机 = 先刹车再停（Ctrl-C / 安全停机 / 正常结束）

以前结束时直接 stop（mode 0，三相断开）：电机不出力，靠摩擦停下。4438 摩擦小（约 0.1 Nm），转得快时 Ctrl-C 之后还会自由转一段，看起来像"Ctrl-C 没用"。现在 `brake_then_stop()`（`control.hpp`）：先 **brake**（mode 15，三相短路，阻力和速度成正比）直到两台都 < 0.02 r/s 或 `--brake`（默认 0.5 s）用完，再 stop（确认 mode 0），所以之后还能用手转。`--brake 0` = 只 stop（以前的做法）。go-zero 的"按 Enter 继续"提示现在按 Ctrl-C 会立刻取消。

⚠️ 这是软件停机：程序崩溃 / 被 kill -9 / 电脑卡死时不会执行。真正的急停要靠**硬件断电**（电源开关或急停按钮切断电机供电）。电机自带的 watchdog（`motor_tool timeout`）还没接进控制循环。

### 所有 mode 都 pipelined（2026-11-12 起默认，1 kHz）

`motor_tool control` 现在**默认用 pipelining**（`run_control_pipelined()`），position / velocity / torque / impedance 都一样；配置文件 `[common]` 里 `rate = 1000`、`pipeline = 1`。`--sequential` 或 `pipeline = 0` = 以前的"发一收一"（`run_control()`，没有改）。安全检查、判定标准（PASS / WARN / FAIL）对所有 mode 一样。

| mode | 每帧发什么 | 帧长（float32） | 用到的状态 |
|---|---|---|---|
| position | 位置 + max-tqe（vel-max / acc 在电脑上限速，`--shape host`） | 20 byte | 只用于安全检查和记录 |
| velocity | 位置 NaN + 速度（host 按 `--vel-ramp` 限制加速度，靠近限位刹车）+ max-tqe | 24 byte | 最新状态用于刹车 |
| torque | `command_tqe_min`（默认，`--tqe-frame min`）：只有力矩、kp scale 0、kd scale 0（0x022–0x024）。`short` = 再加位置 NaN、速度 0（32 byte），`full` = `command_tqe`（48 byte） | **24 byte** | 最新速度用于 `--kd` 阻尼 |
| impedance | 同 torque：τ = kp(q_d − q) + kd(v_d − v)，限幅，kp / kd 在 `--gain-ramp` 内升上来 | 24 byte | 最新位置和速度（1–2 个周期前） |

⚠️ 32 byte 帧在 1 kHz 时（position 每帧发 0x028 / 0x029 的那次）丢失约 0.6–0.7%，所以 torque / impedance 在 1 kHz 可能按判定标准 FAIL（丢失 > 0.2%）。

- 1 kHz torque 用 32 byte 帧（±0.2 / ±0.4 Nm，10 s）：ID 1 丢失 0.38%（FAIL），ID 2 0.05% → 改成 24 byte 的 `command_tqe_min`。为什么可以不写位置 / 速度：位置模式的力矩 = kp scale·kp·（位置误差）+ kd scale·kd·（速度误差）+ 前馈力矩，两个 scale 都是 0 时位置和速度目标不起作用。**不动的检查**（2026-11-12，力矩 0，两台各 5 帧）：电机接受，没有 register error，测到的力矩只是噪声（±0.03 Nm），位置不变，stop 之后确认 mode 0。注意：回复里的 mode 比命令慢一帧（stop → 3 → 10；发 stop 的那一帧回复还是 10），所以停机后会再 query 确认。还没有在有力矩时测过。

### 方案 A：电脑端限速（`--shape host`，pipelined position 的默认）

电机的速度 / 加速度上限（0x028 / 0x029）只在带着它们的那一帧起作用，每帧都发的话帧是 32 byte，1 kHz 丢失多（0.6–0.7%）。所以改成**在电脑上限速**：每个周期先把目标经过 `SetpointShaper`（`control.hpp`）——命令速度不超过 vel-max、每周期最多变化 acc × dt、按"还能刹住"的速度接近目标（离散时间公式 v²/2a + v·dt/2 = 剩余距离），到达时速度刚好为 0、不过冲；目标自己的速度作为前馈，所以限制以内的正弦没有额外滞后。vel-max / acc 来自 `[position]` 配置（和以前一样），但**不再发给电机**；每帧只发位置 + max-tqe = **20 byte**。`--shape motor` = 以前的做法（每帧发 0x028 / 0x029）。单元测试：20° 一步 0.472 s（理论 0.471 s）、峰值 0.2354 r/s（理论 0.2357）、过冲 < 1e-6 r、正弦误差 < 0.05°、快目标被限速。

实测（2026-11-12，1 kHz pipelined，`--safe-vel 0.5`）：

| | 正弦 ±20° 0.25 Hz，20 s | 方波 ±10° 0.25 Hz，10 s |
|---|---|---|
| 丢失 ID 1 / ID 2 | 0.11% / 0.025%（全部发现） | 0.16% / 0.03%（全部发现） |
| 状态 age p99 ID 1 / ID 2 | 5 / 2 | 5 / 2 |
| 位置误差 RMS（对目标） | 1.35° / 1.27° | 5.8° / 6.0°（方波跳变时目标瞬间离开，算进去了） |
| 运动 | 正常 | 每步平滑走完（~0.47 s），不超速，停在离目标 0.1–0.6°（摩擦） |
| 结论 | FAIL：只有一次发送晚 733 µs（macOS 调度卡顿），其余 PASS / WARN | **PASS WITH WARNINGS** |

对比每帧都发 0x028 / 0x029（32 byte）：1 kHz 丢失 0.60% / 0.72%。报告现在也打印"对实际发出的命令"的误差。

### Velocity mode：实测和电脑端速度修正（2026-11-12，1 kHz pipelined）

**速度怎么测**：用**位置变化**算真实速度（50 ms 内位置差 ÷ 时间），不用电机报的速度。位置分辨率 0.088°（1/4096 圈），所以 50 ms 窗口的速度有约 ±1.8 deg/s 的量化误差。

**发现 1：平均速度有比例偏差，原因是电机读到的速度不准，不是速度环没跟上。** 方波 ±30 deg/s、0.25 Hz、20 s，不修正（`--trim-ki 0`），恒速段后半段的平均值：

| | 目标 | 电机报的速度 | 真实速度（从位置算） | 误差 |
|---|---|---|---|---|
| ID 1（4438）正向 / 反向 | +30.0 / −27.8 | +30.0 / −27.7 | +27.8 / −26.3 | 慢约 7% / 5% |
| ID 2（5047）正向 / 反向 | +30.0 / −27.8 | +30.0 / −27.6 | +32.2 / −30.2 | 快约 7% / 9% |

- 电机报的速度平均值正好等于目标，所以电机的速度环是按它**自己读到的速度**调好了，偏差在速度读数本身（比例偏差，两台方向相反）。
- 更正：之前说"5047 速度读数整体偏 +0.023 r/s"，那是数据少的时候的结论，这次的数据不支持。
- 位置和速度哪个对，从外面分不出来（要用量角器量一下才能确认）。位置控制、限位、go-zero 都用位置，所以按位置修正，能让所有 mode 保持一致。

**电脑端修正（实验程序 [experiments/velocity_trim.cpp](experiments/velocity_trim.cpp)，库和 `motor_tool` 没改）**：
- 和 `motor_tool control --mode velocity` 是同一个 pipelined velocity 循环（24 byte 帧、host ramp、限位前刹车、所有安全检查、Ctrl-C = 刹车 + 停），只是通过 trajectory callback 改了目标速度。
- 修正方法：只在稳定段比较。稳定段 = 没被 host ramp 或刹车限制、方向不变、|目标| ≥ 3.6 deg/s；比较时目标往后移 20 ms，对应电机的响应延迟。
- 修正公式：`修正 += ki × (目标 − 真实速度) × dt`，每台电机的正向和反向分开记，最多 ±10 deg/s；发出去的命令 = 目标 + 修正。
- 不在 CMake 里，要单独编译：
  ```bash
  cmake --build build
  c++ -std=c++17 -O2 -Wall -Wextra -Iinclude -Iexamples experiments/velocity_trim.cpp build/libhightorque.a -o build/velocity_trim
  ./build/velocity_trim --id 1,2 --profile sine --amp 30 --freq 0.25 --duration 20 --rt     # --trim-ki 0 = 只测不修正
  ```

结果（后半段平均误差，deg/s）：

| | ID 1 正向 / 反向 | ID 2 正向 / 反向 |
|---|---|---|
| 方波，不修正 | +2.2 / −1.5 | −2.2 / +2.4 |
| 方波，ki 2（默认） | +0.2 / −0.2 | +0.2 / 0.0 |
| 方波，ki 0.5 | −0.2 / −0.7 | −0.3 / +0.1（前半段收敛慢，没有更好） |
| 正弦，ki 2 | 0.0 / +0.1 | 0.0 / 0.0 |

修正量最后约为：ID 1 +2–3 / −2 deg/s，ID 2 −1.5–3 / +0.5–2 deg/s。离线模拟（4438 型慢 15%、5047 型快 3 deg/s，20 ms 延迟）结果一致：误差到约 0，没有振荡。

**发现 2：恒定命令下速度时快时慢，修正解决不了。** 命令 30 deg/s 时，实际速度在约 15 到 45–50 deg/s 之间摆动，RMS 方波 7–11 deg/s、正弦 5.5–8 deg/s，修正前后都一样。
- 这个波动**每个周期在同一相位重复**。例如 ID 1 每次正向换向后约 0.5 s 冲到 40–50 deg/s，约 1.5 s 时（位置约 +29°）只有 15–20 deg/s。
- 电机报的速度在同样的时刻也变慢，说明是真的变慢了。
- 可能的原因有两个：跟位置有关（齿轮 / 皮带 / 负载）；或者换向后速度环在振荡。每 0.5 s 打印一次分不出来，要记录每个回复才能区分（还没做）。
- 修正是故意做慢的（只修平均值），对这种快的波动没有用，修正量本身也会跟着摆约 ±1–3 deg/s。

**结论**：
- 平均速度可以用电脑修正到 < 1 deg/s。
- 速度波动来自电机这一端，等新电机 / 新固件再用同样的命令测。
- 实际使用时速度命令应该是平滑的，正弦比方波更接近实际情况，方波不需要专门调。
- 丢包和其它 mode 一样：ID 1 0.10–0.18%、ID 2 0.015–0.055%，丢的全部在运行中被发现，最长 2 个周期没有新状态，CAN 0 错误 → WARN 级别。

### Pipelining：判定标准（`--pipeline` 的报告）

pipelining 本来就不等回复，所以"回复晚到下一条命令之后"是正常的，不能用"发一收一"的标准（`check_link.py --trace` 的 NOT OK 对 pipelined 运行不适用）。每一项给出 **PASS / WARN / FAIL**，总结论取最差的一项：

| # | 检查 | PASS | WARN（测试时可以接受，要能解释） | FAIL |
|---|---|---|---|---|
| 1 | 运行结束方式 | 跑完或 Ctrl-C | — | 安全停机（限位 / 超速 / fault / 丢回复） |
| 2 | CAN / adapter | err / warn / busoff = 0，没有 ERR 行 | — | 任何 CAN 错误或 ERR 行 |
| 3 | 固定时间表 | 每次发送最多晚 25% 周期 | 有晚于 25% 的 | 任何一次晚于 50% 周期 |
| 4 | 每台丢失（请求数 − 回复数，精确） | 0 | ≤ 0.2%，并且全部在运行中被发现（重新对齐） | > 0.2%，或有没被发现的 |
| 5 | 每台状态 age（控制用的状态是几个周期前的请求的回复） | p99 ≤ 2 周期 | p99.9 ≤ 8 周期 | 更大 |
| 6 | 每台最长连续"没有新状态" | ≤ 2 周期 | < `--max-stale` | ≥ `--max-stale`（会停机） |

另外只打印、不判定：回复延迟 p50 / p99 / max（丢失后最多 20 ms 内配对是错位的，所以 p99 / max 会偏大）、没有新回复的周期数。按这个标准：800 Hz pipelined（0 丢失、age 1–2）= PASS；1 kHz pipelined（ID 1 0.14–0.18% 丢失、age p99 4）= PASS WITH WARNINGS。

### 丢包 / 迟到检测

每次 `motor_tool control` 结束都会打印 link 报告，`--log` 的 CSV 也可以事后用脚本检查：

```bash
python3 tools/check_link.py /tmp/pos.csv            # 一个或多个 log；--hist 打印往返时间直方图
```

| 项目 | 意思 | 不是 0 时说明 |
|---|---|---|
| no reply in time | 这个周期的回复没在 timeout（约 0.9 个周期）内到 → 控制用了上一个状态（STALE） | — |
| = **late** | 回复其实到了，只是晚了（driver 在后面的周期发现并丢掉它） | **时序问题**：往返时间比周期长 → 降频率或做 pipelining |
| + **never seen** | 回复一直没出现 | **bus 问题**（终端、接线、sample point），adapter 通常也会报 err / warn |
| longest run | 最多连续几个周期 STALE（到 3 就两台一起停） | |
| round trip | 发出所有命令 → 收到所有回复的时间，p50 / p99 / p99.9 / max，以及超过一个周期的次数 | headroom 为负 = 这个频率放不下 |
| adapter | 跑完后的 `can status`（每次启动时 CAN 重启、错误清零，所以就是这一次的） | err / warn / busoff ≠ 0 = bus 上有错误帧 |

验收：每台 no reply in time = 0，adapter err / warn / busoff = 0（README "CAN 丢包"）。脚本退出码：0 = OK，1 = 有丢失或迟到。

**精确计数和 trace（2026-11-12）**：回复里没有序号，所以"迟到"和"丢失"只靠周期内的超时分不清。现在 driver 按电机 ID 精确计数"发出的、要求回复的帧"和"收到的帧"（`Fdcanusb::requests_sent()` / `replies_received()`），控制循环结束后再等 100 ms 收完迟到的回复：**请求数 − 回复数 = 真的没到的回复**。`--trace FILE`（`motor_tool control` 和 `rate_test`）记录每一帧的发送 / 接收时间，`python3 tools/check_link.py --trace FILE` 按顺序配对（第 k 个回复 = 第 k 个请求），算出每个回复的真实延迟、多少回复晚到下一条命令发出之后，以及丢失发生在哪里。

**实验：电机会不会因为"新命令到了、旧回复还没发"而丢回复？**（只 query，stop 模式，5 秒，`rate_test --query --rate 2000 --timeout-us 200`，每条命令都在上一个回复之前发出）

| 实验 | 每台请求 | 回复 | 丢失 | 晚到下一条命令之后 |
|---|---|---|---|---|
| 300 Hz（基准） | 1503 | 1503 | 0 | 0 |
| 2000 Hz，两台 | 10002 | 10002 | 0 | ~10000 |
| 2000 Hz，只 ID 2 | 10002 | 10002 | 0 | 6425 |
| 2000 Hz，两台，48 byte 帧（超过 bus / USB 能力，延迟涨到 7–15 ms） | 8877 | 8877 | 0 | 8876 |

→ 至少在 stop 模式下，**电机对每一条命令都按顺序回复，重叠时只是晚，不会丢**。mode 10（工作模式）下还要用 `--trace` 再确认。800 / 1000 Hz 的 position 控制在顺序结构下不行：往返中位数约 1.28 ms，已经比周期长，几乎每个周期都 STALE，连续 `--max-missed`（默认 3）次就停机；要更快需要 pipelining。前几个周期（stop → mode 10，第一帧 64 byte）现在等最多 20 ms（`startup_cycles` / `startup_timeout_s`），不会一开始就停。

**迟到的 root cause（2026-11-12，position 500 Hz，20 秒，`--trace`）**：
- 精确计数：两台各 9998 个请求、9998 个回复，**0 丢失**；12 次"迟到"全部后来到了。
- 每台的延迟（请求写出 → 回复读到）：ID 1 p50 1061 / p99 1843 / max 3191 µs，ID 2 p50 1195 / p99 1836 / max 3957 µs。
- **慢回复几乎从不两台同时出现**（> 1.8 ms：只 ID 1 146 次、只 ID 2 144 次、两台同时 1 次 = 第一帧）→ 不是 USB / host / 整条 bus 的问题（那样会同时拖慢两台）。
- **慢回复锁定在 10 ms 周期上**：相邻两个慢回复之间的间隔 412 / 413（ID 1）和 455 / 455（ID 2）是 10 ms 的整数倍。成串出现（ID 1 每约 0.5 s 一串、ID 2 每约 2 s 一串），是我们 500 Hz 的发送时刻和电机内部 100 Hz 任务的相位慢慢对上又错开（拍频），两台电机各有自己的时钟，所以节奏不同。慢回复比中位数多约 0.5–0.65 ms，最多约 2–2.8 ms。
- → **原因在电机固件：内部每 10 ms 有一个任务，碰上时 CAN 回复被推迟约 0.6 ms。** 不是反射：反射会造成 CAN 错误帧和重发（adapter 的错误计数会动，重发的代价是一帧时间约 0.1–0.2 ms，而且是随机的），这里 adapter 全程 err = 0，延迟又是严格周期性的。所以**加终端电阻不会改善这个**（仍然建议加，为了余量）。
- 1 kHz：除了这个 10 ms 的长尾，**基础往返（中位数）本身就约 1.06–1.2 ms**（USB Full Speed + adapter 文字协议 + CAN + 电机处理），已经比 1 ms 长，所以"发 → 等 → 发"的顺序结构不可能到 1 kHz。要 1 kHz 必须 pipelining：每 1 ms 准时发，回复晚 1–2 个周期到、用最新的那个。重叠时电机不会丢回复（stop 模式 2000 Hz 实验 0 丢失；这次 mode 10 也有 6 次回复晚到下一条命令之后，0 丢失）。

**driver 的改动（2026-11-12）**：之前迟到的回复如果正好在下一个命令发出之后才被读到，会被当成新命令的回复（数据其实旧了一个周期）。现在 `Fdcanusb::transact()` 给每个收到的帧打时间戳，**发出后 200 µs 内读到的帧**（不可能是新命令的回复，实测最短往返约 500 µs）和发出前就在排队的帧，都算 late 并丢掉，按电机 ID 计数（`Fdcanusb::late_replies(id)`）。

## Impedance control（host 端，方案 B）⚠️ 部分验证（5047 弹簧有效，4438 还没推过）

两台电机都是旧 firmware，**mode 21（电机内部的 PD，case 8）会被拒绝**。所以弹簧-阻尼在 host 端每个周期算一次，再用 case 4 当成纯力矩发出去：

```
tau = kp * (target - offset) - kd * velocity        然后限幅到 ±max_tqe
```

- `offset`：离存好的零位的角度（和 go-zero 一样，按 ±180° 折叠），单位 turn。`target` 也是相对零位。
- `kp` 单位 Nm/turn（1 Nm/turn ≈ 0.0028 Nm/deg），`kd` 单位 Nm/(turn/s)。力矩是减速器输出端的真实力矩（已乘 k）。
- 库函数：`run_control()`，mode = `ControlMode::Impedance`（`include/hightorque/control.hpp`，见下面"Host 控制循环"）。`motor_tool impedance ...` 等于 `motor_tool control --mode impedance ...`。命令行：

```bash
# 先回零位（impedance 只在离 target 10° 以内才开始）
./build/motor_tool go-zero --id 1,2 --model M4438_32,M5047_36
# kp = kd = 0：力矩 0，电机可以用手推（检查 300 Hz 工作模式下的丢包 / 往返）
./build/motor_tool impedance --id 1,2 --model M4438_32,M5047_36 --duration 20 --rt
# 弹簧：每台自己的 kp / kd，力矩最多 0.3 Nm，记录到 CSV
./build/motor_tool impedance --id 1,2 --model M4438_32,M5047_36 --kp 2,2 --kd 0.02,0.02 --max-tqe 0.3 \
    --duration 20 --rt --log /tmp/imp.csv
```

选项：`--kp --kd --target-deg --max-tqe`（一个值 = 所有电机，或每个 ID 一个值），`--rate 300`、`--duration 10`（0 = 直到 Ctrl-C）、`--ramp 1`（kp / kd 在 1 秒内从 0 升到设定值）、`--max-vel 0.5`、`--max-start-deg 10`、`--log file.csv`、`--rt`。

**安全检查**（任何一台出问题 → 两台一起 stop，并确认 mode 0）：

- 开始前（不发任何命令）：零位存在、有 hardcoded 限位、电机有回复、在限位内、离 target 不超过 `--max-start-deg`、target 在限位内、gains ≥ 0。
- 每个周期：超出限位、`|速度| > --max-vel`、电机报 fault、**连续 3 个周期没回复**。
- 回复 timeout 约等于一个周期（300 Hz 时 3 ms）。丢回复的周期标成 `STALE` 并计数；健全的 bus 必须是 0（见"CAN 丢包"），不能靠重试。
- ⚠️ 被 `kill -9` 或崩溃时，电机会**保持最后一个力矩**（case 4 不限速）。Ctrl-C / SIGHUP / SIGPIPE 会正常 stop。电机 watchdog（`motor_tool timeout`）还没接进来：它会把 stop 变成刹车，影响用手掰（set-zero），要先弄清楚怎么恢复。

**真机测试（2026-11-11，300 Hz，`--rt`，`--max-tqe 0.3`，用手推）**：

> ⚠️ 2026-11-12 发现：这些测试里的**测量力矩**被多乘了一次 k（`Motor::decode()` 已经乘过）：5047 的读数是实际的 0.80 倍，4438 是 0.56 倍。命令力矩、摩擦估计（按命令力矩算）和相关系数不受影响。已修复。


| kp / kd | 周期 | 丢失 | 结果 |
|---|---|---|---|
| 0 / 0 | 6000 | 0 / 0 | 力矩 0，测量噪声约 ±0.02 Nm（sd 0.015） |
| 0 / 0.02 | 9000 | 0 / 0 | 阻尼太小：0.36 r/s 时只有 0.007 Nm，被噪声盖住 |
| 0 / 0.2 | 3991 | 0 / 0 | 阻尼方向正确（cmd 和速度相关 −0.83），没有振荡；ID 2 被推过 +90° → **限位停机正确触发**，两台都确认 stop |
| 5 / 0.2 | 9000 | 0 / 0 | **ID 2 弹簧有效**：测量力矩跟命令相关 0.98；松手后自己往回走（+38° → +13°，+53° → +12.5°），但停在 12–18°（弹簧 0.17–0.26 Nm < 静摩擦）；没有振荡 |
| 10 / 0.2 | 8896 | 0 / 0 | **ID 1（4438）**：拉到 −48° / −84° / +71° 松手都自己回来（最快约 0.38 r/s），过零一点后停在 **±3–5°**，没有持续振荡；停住时弹簧 0.08–0.10 Nm ≈ 4438 的静摩擦。**ID 2（5047）**：在 +15° 和 +66° 卡住，弹簧已顶到 0.3 Nm 上限（测到约 0.25 Nm）也不动 → 5047 起步约需 0.3 Nm（= 上限），动起来后约 0.17–0.2 Nm 才停。最后被拉太快（> 0.5 r/s）安全停机 |
| 20 / 0.2 | 9000 | 0 / 0 | 这次没人推，没有数据 |
| 10 / 0.2，`--max-tqe 0.3,0.5`（ID 2 提到 0.5 Nm） | 9000 | 0 / 0 | **ID 2 明显改善**：从 24–54° 松手，8 次都回到 **±1.3–4.8°**，最快 0.35 r/s，没有振荡。从零点附近慢慢推到 7–8° 松手不会回来（弹簧约 0.2 Nm < 起步约 0.3 Nm）→ 静止死区约 ±7–8°。用户手感：ID 1（4438）回零很准 |

- 用户手感（kp 5）：ID 2 在零点 ±10° 内推了没反应；拉到 ±30° 松手只回到 ±10°，不回零点。
- 死区 ≈ 静摩擦 / kp，但**只要弹簧力矩顶到 `--max-tqe` 还小于起步摩擦，加大 kp 也没用**：5047 起步约 0.3 Nm，所以 5047 要回得更近，需要提高上限（例如 0.5 Nm）或者做摩擦补偿。4438 摩擦小（约 0.1 Nm），kp 10 就能回到 ±3–5°。
- kd 0.2 的阻尼（约 0.04 Nm）远小于摩擦，手感上感觉不到。
- 每次之间用 `go-zero --id 1,2` 同步回零（1–3 秒，误差 0.1–0.3°）。

**控制频率**（kp = kd = 0，电机在 mode 10 但力矩 0，两台一起，`--rt`；`motor_tool impedance --type float|int16 --rate N`；回复 timeout = 0.9 个周期）：

| 频率 | float（力矩帧 48 byte） | int16（24 byte） |
|---|---|---|
| 300 Hz | 0 丢失（多次测试合计约 45,000 个周期） | — |
| 400 Hz | 8000 个周期里 ID 2 迟到 1 次 | — |
| 500 Hz | 每 10,000 个周期迟到 1–6 次 | 每 10,000 个周期迟到 1–2 次 |
| 600–700 Hz | 约 3–4% 迟到 | 600 Hz：一开始就连续 3 次迟到 → 停机；700 Hz 约 0.1% |
| 1000 Hz | 约 3–4%，只跑到约 970 Hz | 约 9%，只跑到约 930 Hz |

- 每次跑完 adapter 都是 `err=0 warn=0`，**所以这些"丢失"全是回复迟到**（超过 timeout），不是坏帧。原因是电机在工作模式下回复偶尔会慢到约 1.5–2 ms（见"关于 1 kHz"），加上"发 → 等回复 → 再发"的顺序结构。
- 按"不能丢包"的标准，**现在的顺序结构最高就是 300 Hz**。
- 要更快只能做 pipelining（不等回复就发下一帧，迟到的回复用在下一个周期，标成 stale 并计数），见 `docs/pipelining_design.md`，还没实现。
- 刚从 stop 切到 mode 10 的头几个周期回复更慢（int16 600 Hz 一开始就停机），以后可以给前几个周期更长的 timeout。

**已知限制**：

- 延迟：力矩用的是上一个周期的状态，300 Hz 时总延迟约 3.3 ms 周期 + 1.2 ms 往返，所以 kd / kp 不能很大。从小往上加。
- **没有摩擦补偿**：5047 的起步摩擦约 0.5–1 Nm。比摩擦还弱的弹簧推不回来（需要先做摩擦辨识）。
- 下周会换新电机和新 firmware；如果新 firmware 支持 mode 21，应该改用电机内部的 PD（case 8），这里的 host 端版本只是过渡。
