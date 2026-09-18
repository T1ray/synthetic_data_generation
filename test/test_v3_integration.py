import gc
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from synthetic_data_generation.bag_roundtrip import roundtrip_bag
from synthetic_data_generation.ground_truth import annotation_path_for
from synthetic_data_generation.pointcloud_codec import decode_cloud, encode_cloud
from synthetic_data_generation.processor import ProcessingContext
from synthetic_data_generation.scenario import parse_scenario
from synthetic_data_generation.smoke_test import CLOUD_TOPIC, STRING_TOPIC, make_padded_point_cloud, read_bag


def scan(sequence):
    angles=np.linspace(-.2,.2,9); xyz=20*np.column_stack((np.cos(angles),np.sin(angles),np.zeros(9)))
    cloud=make_padded_point_cloud(xyz,sequence=sequence); decoded=decode_cloud(cloud)
    decoded.points["ring"][:]=7; decoded.points["timestamp"][:]=np.arange(9)
    decoded.points["x"][0,4]=decoded.points["y"][0,4]=decoded.points["z"][0,4]=0
    encode_cloud(cloud,decoded); return cloud


def create_bag(path):
    writer=rosbag2_py.SequentialWriter(); writer.open(rosbag2_py.StorageOptions(uri=str(path),storage_id="sqlite3"),rosbag2_py.ConverterOptions("",""))
    writer.create_topic(rosbag2_py.TopicMetadata(name=CLOUD_TOPIC,type="sensor_msgs/msg/PointCloud2",serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name=STRING_TOPIC,type="std_msgs/msg/String",serialization_format="cdr"))
    for index in range(3):
        writer.write(CLOUD_TOPIC,serialize_message(scan(index)),2_000_000_000+index*100_000_000)
        writer.write(STRING_TOPIC,serialize_message(String(data=str(index))),2_050_000_000+index*100_000_000)
    writer.close()


def scenario(path):
    return parse_scenario({"schema_version":3,"scenario_id":"integration-v3","seed":42,
      "source":{"pointcloud_topic":CLOUD_TOPIC},"frames":{"start_index":0,"end_index":2},
      "zero_slot_recovery":{"enabled":True,"ring_field":"ring","timestamp_field":"timestamp","timestamp_unit":"relative_ticks","min_valid_samples_per_ring":4,"interpolation":"linear","allow_extrapolation":False,"max_angular_error_deg":.1},
      "sensor_effects":{"no_return_encoding":"zero_xyz",
       "range_noise":{"enabled":True,"model":"gaussian","base_sigma_m":.001,"sigma_per_meter":0,"incidence_sigma_scale":0,"min_range_m":.2,"max_resample_attempts":4},
       "dropout":{"enabled":True,"base_return_probability":1,"distance_reference_m":30,"distance_exponent":1,"incidence_exponent":1,"min_return_probability":1},
       "intensity":{"enabled":True,"model":"empirical","field_name":"intensity","range_bin_m":5,"min_samples_per_bin":1,"incidence_exponent":1,"additive_sigma":0,"fallback":"preserve_original","on_missing":"error"}},
      "track":{"reference_frame":"map","model":{"type":"polyline","points_m":[[0,0,0],[20,0,0]]},"up_hint":[0,0,1]},
      "ego_motion":{"source":"trajectory_csv","path":str(path),"reference_frame":"map","lidar_frame":"synthetic_lidar","timestamp_unit":"nanoseconds","max_interpolation_gap_ms":100},
      "visualization":{"enabled":True,"marker_topic":"/synthetic/markers","point_size_m":.05,"marker_lifetime_sec":.2},
      "objects":[{"id":"track-box","class_name":"obstacle","geometry":{"type":"box","dimensions_m":[2,4,2]},
       "placement":{"type":"track_relative","longitudinal_m":10,"lateral_m":0,"height_m":-1,"rpy_track_deg":[0,0,0]},
       "temporal":{"active_frames":{"start_index":0,"end_index":2},"motion":{"type":"static"}},
       "material":{"reflectivity":.5,"return_probability_scale":1},"visualization":{"color_rgba":[1,0,0,1]}}]},base_dir=path.parent)


def test_full_v3_bag_is_deterministic_and_consistent(tmp_path: Path):
    trajectory=tmp_path/"trajectory.csv"
    trajectory.write_text("timestamp_ns,x_m,y_m,z_m,qx,qy,qz,qw\n2000000000,0,0,0,0,0,0,1\n2100000000,.1,0,0,0,0,0,1\n2200000000,.2,0,0,0,0,0,1\n",encoding="utf-8")
    input_bag=tmp_path/"input"; create_bag(input_bag)
    outputs=[tmp_path/"out-a",tmp_path/"out-b"]; cloud_bytes=[]; json_bytes=[]
    for output in outputs:
        result=roundtrip_bag(input_bag,output,progress_every=0,context=ProcessingContext.from_scenario(scenario(trajectory)))
        topics,records=read_bag(output)
        assert result.input_message_count==6 and result.output_message_count==9
        assert topics["/synthetic/markers"]=="visualization_msgs/msg/MarkerArray"
        clouds=[record.message for record in records if record.topic_name==CLOUD_TOPIC]
        markers=[record for record in records if record.topic_name=="/synthetic/markers"]
        assert len(clouds)==len(markers)==3
        assert all(cloud.width==9 and cloud.point_step==32 for cloud in clouds)
        cloud_bytes.append([bytes(cloud.data) for cloud in clouds]); json_bytes.append(annotation_path_for(output).read_bytes())
        annotations=[json.loads(line) for line in annotation_path_for(output).read_text(encoding="utf-8").splitlines()]
        assert all(item["zero_slot_count"]==1 and item["recovered_direction_count"]==1 for item in annotations)
        assert all(item["synthetic_returns_from_zero_slots"]==1 for item in annotations)
        assert all(item["sensor_effects"]["range_noise_enabled"] for item in annotations)
        del records; gc.collect()
    assert cloud_bytes[0]==cloud_bytes[1] and json_bytes[0]==json_bytes[1]
