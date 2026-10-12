import psycopg2
import numpy as np

from assignment_object_detection.database import DatabaseStore
from assignment_object_detection.queries import nearest_object_query, region_query, class_histogram_query, object_observations_query


def test_queries_basic_sql_smoke(conn):
    try:
        store = DatabaseStore(conn)
        store.init_schema()
        obj_id = store.create_object("chair", 1.0, 2.0, 0.0, np.ones(512, dtype=np.float32), 1, 5)
        store.insert_observation(obj_id, 2, 1.0, 2.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.2, 1.0, [0, 0, 10, 10], 0.9, np.ones(512, dtype=np.float32))
        with conn.cursor() as cur:
            rows = nearest_object_query(cur, "chair", 1.0, 2.0)
            assert len(rows) == 1
            rows = region_query(cur, 0.0, 10.0, 0.0, 10.0)
            assert len(rows) == 1
            rows = class_histogram_query(cur)
            assert len(rows) >= 1
            rows = object_observations_query(cur, obj_id)
            assert len(rows) == 1
    finally:
        conn.rollback()
