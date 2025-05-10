import json
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# --- Load Ground Truth JSON ---
def load_ground_truth(json_path):
    with open(json_path) as f:
        data = json.load(f)
    return [np.array([cone['x'], cone['y'], cone['z']]) for cone in data['cones']]

# --- Read PoseArray messages from rosbag2 ---
def load_detected_centers(bag_path, topic='/cone_centres'):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    msg_type = None
    centers = []

    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            if msg_type is None:
                msg_type_str = reader.get_all_topics_and_types()[0].type
                msg_type = get_message(msg_type_str)
            msg = deserialize_message(data, msg_type)
            for pose in msg.poses:
                centers.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
    return centers

# --- Compare with Tolerance ---
def compute_match_stats(detected, ground_truth, threshold=0.5):
    matched = 0
    used = set()
    for d in detected:
        for i, gt in enumerate(ground_truth):
            if i in used:
                continue
            if np.linalg.norm(d - gt) <= threshold:
                matched += 1
                used.add(i)
                break
    total = len(ground_truth)
    return {
        'matched_cones': matched,
        'total_ground_truth': total,
        'match_percentage': round((matched / total) * 100, 2) if total else 0.0
    }

# --- Paths ---
json_path = '/home/jarred/git/drive_to_survive/src/ground_truth/cone_centers.json'
bag_path = '/home/jarred/git/drive_to_survive/validation/rosbags/lidar_cone_centers'

# --- Run ---
ground_truth = load_ground_truth(json_path)
detected = load_detected_centers(bag_path)
results = compute_match_stats(detected, ground_truth)

print("✅ Evaluation Results:")
print(results)
