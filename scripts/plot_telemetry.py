import serial
import json
import time
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
SERIAL_PORT = "COM5"      
BAUD_RATE   = 115200
CAPTURE_SEC = 20          # Capture 20 seconds (40 frames at 500 ms)

ts_list, sp_list, out_list, err_list = [], [], [], []
p1_list, p2_list, p3_list = [], [], []
wcet_latest = [0, 0, 0]

print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud for {CAPTURE_SEC}s...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(1.5)
ser.reset_input_buffer()

start_time = time.time()
t0_ms = None

try:
    while (time.time() - start_time) < CAPTURE_SEC:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line.startswith("{"):
            continue
        try:
            data = json.loads(line)
            if t0_ms is None:
                t0_ms = data["ts"]
            ts_list.append((data["ts"] - t0_ms) / 1000.0)
            sp_list.append(data["sp"])
            out_list.append(data["out"])
            err_list.append(data["err"])

            p1, p2, p3 = data["period_us"]
            p1_list.append(p1 / 1000.0)   # ms
            p2_list.append(p2 / 1000.0)   # ms
            p3_list.append(p3 / 1000.0)   # ms
            wcet_latest = data["wcet_us"] # us
            print(line)
        except (json.JSONDecodeError, KeyError):
            continue
finally:
    ser.close()

# --- COMPUTE CPU UTILIZATION (WORST-CASE BOUND) ---
# U = (WCET_1 / T_1) + (WCET_2 / T_2) + (WCET_3 / T_3)
u1 = (wcet_latest[0] / 10000.0) * 100.0
u2 = (wcet_latest[1] / 50000.0) * 100.0
u3 = (wcet_latest[2] / 500000.0) * 100.0
total_cpu = u1 + u2 + u3

print(f"\nCaptured {len(ts_list)} samples.")
print(f"WCET (us): Task1={wcet_latest[0]}, Task2={wcet_latest[1]}, Task3={wcet_latest[2]}")
print(f"Worst-Case CPU Utilization: {total_cpu:.2f}% (T1={u1:.2f}%, T2={u2:.2f}%, T3={u3:.2f}%)")

# --- PLOT 1: CLOSED-LOOP PID RESPONSE ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
ax1.plot(ts_list, sp_list, label="Setpoint (Filtered)", linestyle="--", color="tab:orange")
ax1.plot(ts_list, out_list, label="Plant Output", linewidth=2, color="tab:blue")
ax1.set_ylabel("Control Value (%)")
ax1.set_title("STM32 FreeRTOS Closed-Loop PID Step Response")
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2.plot(ts_list, err_list, label="Tracking Error", color="tab:red")
ax2.axhline(0, color="black", linewidth=0.8, linestyle=":")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Error")
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("pid_response.png", dpi=150)

# --- PLOT 2: REAL-TIME TIMING & WCET ANALYSIS ---
fig2, (bx1, bx2) = plt.subplots(1, 2, figsize=(11, 4.5))

tasks = ["Task 1\n(10 ms)", "Task 2\n(50 ms)", "Task 3\n(500 ms)"]
wcet_ms = [w / 1000.0 for w in wcet_latest]
bars = bx1.bar(tasks, wcet_ms, color=["#2ca02c", "#1f77b4", "#ff7f0e"])
bx1.set_ylabel("WCET (ms)")
bx1.set_title("Worst-Case Execution Time (DWT->CYCCNT)")
bx1.grid(axis="y", alpha=0.3)
for bar, val in zip(bars, wcet_latest):
    bx1.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
             f"{val} us", ha="center", va="bottom", fontsize=9)

bx2.plot(ts_list, [p - 10.0 for p in p1_list], label="Task 1 Jitter (ms)", marker="o", markersize=3)
bx2.plot(ts_list, [p - 50.0 for p in p2_list], label="Task 2 Jitter (ms)", marker="s", markersize=3)
bx2.plot(ts_list, [p - 500.0 for p in p3_list], label="Task 3 Jitter (ms)", marker="^", markersize=3)
bx2.set_xlabel("Time (s)")
bx2.set_ylabel("Period Error vs Target (ms)")
bx2.set_title(f"Scheduling Jitter (Total CPU Util: {total_cpu:.2f}%)")
bx2.legend()
bx2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("rtos_timing_analysis.png", dpi=150)
plt.show()