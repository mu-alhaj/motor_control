import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================
# CONFIG
# ==========================
COM_PORT = 'COM8'
BAUD_RATE = 921600
MAX_POINTS = 500   # fixed width of x-axis

# ==========================
# BUFFERS
# ==========================
bemf = []
half_bus = []
blanking_sig = []

# ==========================
# SERIAL INIT
# ==========================
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

# ==========================
# PLOT SETUP
# ==========================
plt.style.use('ggplot')
fig, ax = plt.subplots(figsize=(10,6))

line_bemf,      = ax.plot([], [], label="BEMF Filtered")
line_halfbus,   = ax.plot([], [], label="Half Bus Voltage")
line_blanking,  = ax.step([], [], label="Blanking", where="post", linestyle="--")

ax.set_title("Live ESC Data")
ax.set_xlabel("Samples")
ax.set_ylabel("Voltage / Signal")
ax.legend()
ax.grid(True)

# FIXED HORIZONTAL RANGE
ax.set_xlim(0, MAX_POINTS)

# ==========================
# UPDATE FUNCTION
# ==========================
def update(frame):
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            return line_bemf, line_halfbus, line_blanking

        parts = line.split('-')
        if len(parts) < 4:
            return line_bemf, line_halfbus, line_blanking

        phase = parts[0].strip()
        blanking = int(parts[1].strip())
        bemf_val = float(parts[2].strip())
        bus_val = float(parts[3].strip())

        # push into buffers
        bemf.append(bemf_val)
        half_bus.append(bus_val)
        blanking_sig.append(3 if blanking else 0)

        # keep buffers to MAX_POINTS
        if len(bemf) > MAX_POINTS:
            bemf.pop(0)
            half_bus.pop(0)
            blanking_sig.pop(0)

        x = range(len(bemf))

        # update line data
        line_bemf.set_data(x, bemf)
        line_halfbus.set_data(x, half_bus)
        line_blanking.set_data(x, blanking_sig)

        # only rescale Y axis
        ax.set_ylim(
            min(min(bemf), min(half_bus), min(blanking_sig)) - 0.1,
            max(max(bemf), max(half_bus), max(blanking_sig)) + 0.1
        )

    except Exception:
        pass

    return line_bemf, line_halfbus, line_blanking

# ==========================
# ANIMATION LOOP
# ==========================
ani = animation.FuncAnimation(
    fig,
    update,
    interval=1,
    blit=False
)

plt.tight_layout()
plt.show()
ser.close()
