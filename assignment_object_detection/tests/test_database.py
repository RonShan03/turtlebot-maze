import numpy as np
import psycopg2
import pytest

from assignment_object_detection.database import DatabaseStore, init_database


def test_schema_initialization(conn):
    init_database(conn)
    with conn.cursor() as cur:
        cur.execute("SELECT to_regclass('semantic_objects')")
        assert cur.fetchone()[0] == "semantic_objects"
        cur.execute("SELECT to_regclass('object_observations')")
        assert cur.fetchone()[0] == "object_observations"


def test_store_and_insert_object(conn):
    store = DatabaseStore(conn)
    store.init_schema()
    obj_id = store.create_object(
        class_name="chair",
        map_x=1.0,
        map_y=2.0,
        map_z=0.0,
        mean_embedding=np.ones(512, dtype=np.float32),
        first_seen_ns=10,
        last_seen_ns=20,
        observation_count=1,
    )
    obs_id = store.insert_observation(
        object_id=obj_id,
        seen_at_ns=15,
        map_x=1.1,
        map_y=2.2,
        map_yaw=0.3,
        camera_x=0.0,
        camera_y=0.0,
        camera_z=0.5,
        camera_qx=0.0,
        camera_qy=0.0,
        camera_qz=0.0,
        camera_qw=1.0,
        bearing_rad=0.1,
        range_m=1.2,
        bbox=[1, 2, 3, 4],
        confidence=0.9,
        embedding=np.ones(512, dtype=np.float32),
    )
    assert obj_id > 0
    assert obs_id > 0
