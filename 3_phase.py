import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from datetime import datetime

# ==========================
# CONFIG
# ==========================
COM_PORT = 'COM8'
BAUD_RATE = 921600
MAX_POINTS = 500
LOG_FILE = "esc_live_log.txt"

# ==========================
# BUFFERS
# ==========================
bemf_a = []
bemf_b = []
bemf_c = []

# ==========================
# SERIAL INIT
# ==========================
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

# ==========================
# LOG FILE INIT
# ==========================
log = open(LOG_FILE, "a", buffering=1)  # line-buffered
log.write("\n===== New session: {} =====\n".format(datetime.now()))

# ==========================
# PLOT SETUP
# ==========================
plt.style.use('ggplot')
fig, ax = plt.subplots(figsize=(10,6))

line_a, = ax.plot([], [], label="A")
line_b, = ax.plot([], [], label="B")
line_c, = ax.plot([], [], label="C")

ax.set_title("Live ESC Data")
ax.set_xlabel("Samples")
ax.set_ylabel("Voltage")
ax.legend()
ax.grid(True)

ax.set_xlim(0, MAX_POINTS)

# ==========================
# UPDATE FUNCTION
# ==========================
def update(frame):
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            return line_a, line_b, line_c

        # save raw line to file
        log.write(line + "\n")

        # parse values
        parts = line.split('-')
        if len(parts) < 3:
            return line_a, line_b, line_c

        a = float(parts[0].strip())
        b = float(parts[1].strip())
        c = float(parts[2].strip())

        bemf_a.append(a)
        bemf_b.append(b)
        bemf_c.append(c)

        # maintain fixed buffer size
        if len(bemf_a) > MAX_POINTS:
            bemf_a.pop(0)
            bemf_b.pop(0)
            bemf_c.pop(0)

        x = range(len(bemf_a))

        line_a.set_data(x, bemf_a)
        line_b.set_data(x, bemf_b)
        line_c.set_data(x, bemf_c)

        ymin = min(min(bemf_a), min(bemf_b), min(bemf_c))
        ymax = max(max(bemf_a), max(bemf_b), max(bemf_c))
        ax.set_ylim(ymin - 0.1, ymax + 0.1)

    except Exception:
        pass

    return line_a, line_b, line_c

# ==========================
# ANIMATION LOOP
# ==========================
ani = animation.FuncAnimation(fig, update, interval=1, blit=False)

plt.tight_layout()
plt.show()

# ==========================
# CLEANUP
# ==========================
ser.close()
log.close()
