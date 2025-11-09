import serial
import matplotlib.pyplot as plt

# ==========================
# CONFIG
# ==========================
COM_PORT = 'COM8'
BAUD_RATE = 921600
LOG_FILE = 'esc_data_log.txt'

# ==========================
# RECORDING MODE
# ==========================
def record_data():
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    print("Recording data. Press Ctrl+C to stop...")
    with open(LOG_FILE, 'w') as f:
        try:
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    print(line)
                    f.write(line + '\n')
        except KeyboardInterrupt:
            print("Recording stopped.")
    ser.close()

# ==========================
# PLAYBACK MODE
# ==========================
def playback_data():
    phases = []
    blankings = []
    bemf_filtered = []
    half_bus = []

    with open(LOG_FILE, 'r') as f:
        for line in f:
            try:
                parts = line.split('-')
                phase = parts[0].strip()
                blanking = int(parts[1].strip())
                bemf = float(parts[2].strip())
                bus = float(parts[3].strip())

                phases.append(phase)
                blankings.append(blanking)
                bemf_filtered.append(bemf)
                half_bus.append(bus)
            except:
                continue

    # Convert blanking to digital signal for visualization (3 when active, 0 when inactive)
    blanking_digital = [3 if b else 0 for b in blankings]

    # Plotting
    fig, ax = plt.subplots(figsize=(10,6))
    ax.plot(bemf_filtered, label='BEMF Filtered', color='blue')
    ax.plot(half_bus, label='Half Bus Voltage', color='orange')
    ax.step(range(len(blanking_digital)), blanking_digital, label='Blanking', color='red', where='post', linestyle='--')

    ax.set_title('BEMF, Half Bus Voltage, and Blanking Signal')
    ax.set_xlabel('Samples')
    ax.set_ylabel('Voltage / BEMF')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()

# ==========================
# MAIN
# ==========================
if __name__ == "__main__":
    mode = input("Record data (r) or playback (p)? ").strip().lower()
    if mode == 'r':
        record_data()
    elif mode == 'p':
        playback_data()
    else:
        print("Invalid mode. Choose 'r' or 'p'.")
