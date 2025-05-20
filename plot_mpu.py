import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

SERIAL_PORT = 'COM8'
BAUD_RATE = 115200
MAX_POINTS = 200

# Data buffers
yaw_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
pitch_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
roll_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

def update(frame, ser, lines):
    while ser.in_waiting:
        line = ser.readline().decode('utf-8').strip()
        parts = line.split(',')
        if len(parts) != 3:
            continue

        try:
            yaw, pitch, roll = map(float, parts)
            yaw_data.append(yaw)
            pitch_data.append(pitch)
            roll_data.append(roll)
        except ValueError:
            continue

    lines[0].set_ydata(yaw_data)
    lines[1].set_ydata(pitch_data)
    lines[2].set_ydata(roll_data)
    return lines

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    fig, axs = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

    # Set Y axis limits for each subplot
    for ax in axs:
        ax.set_ylim(-180, 180)
        ax.set_ylabel("Degrees")
        ax.grid(True)

    axs[0].set_title("Yaw")
    axs[1].set_title("Pitch")
    axs[2].set_title("Roll")
    axs[2].set_xlabel("Samples")

    # Create line objects for each subplot
    line_yaw, = axs[0].plot(yaw_data, label='Yaw', color='tab:blue')
    line_pitch, = axs[1].plot(pitch_data, label='Pitch', color='tab:orange')
    line_roll, = axs[2].plot(roll_data, label='Roll', color='tab:green')

    ani = animation.FuncAnimation(
        fig, update, fargs=(ser, [line_yaw, line_pitch, line_roll]),
        interval=10, blit=True, cache_frame_data=False
    )

    plt.tight_layout()
    plt.show()

    ser.close()

if __name__ == "__main__":
    main()
