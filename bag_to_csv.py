import os
import pandas as pd

bag_folder = 'rosbag2_2025_05_27-21_17_10'  # ← kendi klasör adını buraya yaz
output_csv = 'bag_export.csv'

# ROS 2 verisini .csv'ye çevirmek için sqlite3 kullanacağız
import sqlite3

db_path = os.path.join(bag_folder, 'rosbag2_0.db3')
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

cursor.execute("SELECT name FROM topics")
topics = [row[0] for row in cursor.fetchall()]

frames = []

for topic in topics:
    cursor.execute(f"""
        SELECT timestamp, data
        FROM messages
        JOIN topics ON messages.topic_id = topics.id
        WHERE topics.name = '{topic}'
    """)
    rows = cursor.fetchall()
    for row in rows:
        frames.append({
            'timestamp': row[0],
            'topic': topic,
            'value': float(row[1][8:])  # std_msgs/Float64 için veri ayıklama
        })

df = pd.DataFrame(frames)
df.to_csv(output_csv, index=False)
print(f"Veri {output_csv} olarak kaydedildi.")
