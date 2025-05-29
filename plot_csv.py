import csv
import matplotlib.pyplot as plt

# CSV dosyasının yolu (ROS 2'nin share dizininden okunuyor)
csv_path = 'install/kalman_pkg/share/kalman_pkg/estimates.csv'

# Verileri tutacak listeler
timestamps = []
sensor_pos = []
sensor_vel = []
est_pos = []
est_vel = []

# CSV dosyasını oku
with open(csv_path, mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        timestamps.append(float(row['timestamp']))
        sensor_pos.append(float(row['sensor_pos']))
        sensor_vel.append(float(row['sensor_vel']))
        est_pos.append(float(row['est_pos']))
        est_vel.append(float(row['est_vel']))

# Grafik çizimi
plt.figure(figsize=(12, 6))

# Pozisyon çizimi
plt.subplot(2, 1, 1)
plt.plot(timestamps, sensor_pos, label='Ölçülen Pozisyon', marker='o')
plt.plot(timestamps, est_pos, label='Tahmin Edilen Pozisyon', marker='x')
plt.xlabel('Zaman (s)')
plt.ylabel('Pozisyon')
plt.title('Pozisyon Karşılaştırması')
plt.legend()
plt.grid(True)

# Hız çizimi
plt.subplot(2, 1, 2)
plt.plot(timestamps, sensor_vel, label='Ölçülen Hız', marker='o')
plt.plot(timestamps, est_vel, label='Tahmin Edilen Hız', marker='x')
plt.xlabel('Zaman (s)')
plt.ylabel('Hız')
plt.title('Hız Karşılaştırması')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('kalman_plot.png', dpi=300)
plt.show()
