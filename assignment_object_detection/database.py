"""PostgreSQL helpers for the semantic object-detection assignment."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import numpy as np

from pgvector.psycopg2 import register_vector


def _vector_value(vector: Sequence[float] | np.ndarray) -> list[float]:
    arr = np.asarray(vector, dtype=np.float32).reshape(-1)
    if arr.size != 512 or not np.isfinite(arr).all() or np.linalg.norm(arr) == 0:
        raise ValueError('Embedding must contain 512 finite values with nonzero norm')
    normalized = arr / np.linalg.norm(arr)
    return normalized.astype(np.float32).tolist()


def _vector_from_db(value) -> np.ndarray:
    """Accept pgvector 0.5 Vector, older ndarray adapters, or text results."""
    if hasattr(value, 'to_numpy'):
        return value.to_numpy()
    if isinstance(value, str):
        import json
        value = json.loads(value)
    return np.asarray(value, dtype=np.float32)


def _execute_sql_statements(conn: Any, sql_text: str) -> None:
    with conn.cursor() as cur:
        for statement in sql_text.split(";"):
            stmt = statement.strip()
            if stmt:
                cur.execute(stmt)


def init_database(conn: Any) -> None:
    """Create the pgvector extension and semantic tables if they do not exist."""
    schema_path = Path(__file__).with_name("sql") / "semantic_schema.sql"
    sql_text = schema_path.read_text(encoding="utf-8")
    _execute_sql_statements(conn, sql_text)
    conn.commit()
    if register_vector is not None:
        register_vector(conn)


class DatabaseStore:
    """Thin wrapper around PostgreSQL storage for semantic objects and observations."""

    def __init__(self, conn: Any):
        self.conn = conn

    def init_schema(self) -> None:
        init_database(self.conn)

    def create_object(
        self,
        class_name: str,
        map_x: float,
        map_y: float,
        map_z: float,
        mean_embedding: Sequence[float] | np.ndarray,
        first_seen_ns: int,
        last_seen_ns: int,
        observation_count: int = 1,
    ) -> int:
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO semantic_objects (
                    class_name, map_x, map_y, map_z, observation_count, mean_embedding,
                    first_seen_ns, last_seen_ns
                ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
                RETURNING object_id
                """,
                (
                    class_name,
                    float(map_x),
                    float(map_y),
                    float(map_z),
                    int(observation_count),
                    _vector_value(mean_embedding),
                    int(first_seen_ns),
                    int(last_seen_ns),
                ),
            )
            row = cur.fetchone()
            self.conn.commit()
            if row is None:
                raise RuntimeError("Failed to create semantic object row.")
            return int(row[0])

    def insert_observation(
        self,
        object_id: int,
        seen_at_ns: int,
        map_x: float,
        map_y: float,
        map_yaw: float,
        camera_x: float,
        camera_y: float,
        camera_z: float,
        camera_qx: float,
        camera_qy: float,
        camera_qz: float,
        camera_qw: float,
        bearing_rad: float,
        range_m: float,
        bbox: Sequence[float],
        confidence: float,
        embedding: Sequence[float] | np.ndarray,
        commit: bool = True,
    ) -> int:
        bbox_values = [int(float(v)) for v in bbox]
        if len(bbox_values) != 4:
            raise ValueError("bbox must contain exactly four values: xmin, ymin, xmax, ymax.")

        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO object_observations (
                    object_id, seen_at_ns,
                    map_x, map_y, map_yaw,
                    camera_x, camera_y, camera_z,
                    camera_qx, camera_qy, camera_qz, camera_qw,
                    bearing_rad, range_m,
                    bbox_xmin, bbox_ymin, bbox_xmax, bbox_ymax,
                    confidence, embedding
                ) VALUES (
                    %s, %s, %s, %s, %s,
                    %s, %s, %s,
                    %s, %s, %s, %s,
                    %s, %s,
                    %s, %s, %s, %s,
                    %s, %s
                ) RETURNING observation_id
                """,
                (
                    int(object_id),
                    int(seen_at_ns),
                    float(map_x),
                    float(map_y),
                    float(map_yaw),
                    float(camera_x),
                    float(camera_y),
                    float(camera_z),
                    float(camera_qx),
                    float(camera_qy),
                    float(camera_qz),
                    float(camera_qw),
                    float(bearing_rad),
                    float(range_m),
                    int(bbox_values[0]),
                    int(bbox_values[1]),
                    int(bbox_values[2]),
                    int(bbox_values[3]),
                    float(confidence),
                    _vector_value(embedding),
                ),
            )
            row = cur.fetchone()
            if commit:
                self.conn.commit()
            if row is None:
                raise RuntimeError("Failed to insert object observation.")
            return int(row[0])

    def ingest(self, class_name, position, embedding, seen_at_ns, robot_pose,
               camera_pose, bbox, confidence, bearing_rad, range_m, radius=0.5):
        """Associate and persist object + evidence atomically, serialized across writers."""
        from .deduplication import DetectionCluster, ObjectDeduplicator
        vector = _vector_value(embedding)
        point = np.asarray(position, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all() or radius <= 0:
            raise ValueError('Invalid position or association radius')
        try:
            with self.conn.cursor() as cur:
                cur.execute('LOCK TABLE semantic_objects IN SHARE ROW EXCLUSIVE MODE')
                cur.execute('''SELECT object_id, map_x, map_y, map_z, mean_embedding,
                    observation_count, first_seen_ns, last_seen_ns FROM semantic_objects
                    WHERE class_name=%s AND
                    power(map_x-%s,2)+power(map_y-%s,2)+power(map_z-%s,2)<=%s
                    ORDER BY object_id FOR UPDATE''', (class_name, *point.tolist(), radius**2))
                candidates = cur.fetchall()
                clusters = [DetectionCluster(class_name, np.array(row[1:4]),
                            _vector_from_db(row[4]), row[5], row[6], row[7]) for row in candidates]
                merged, created = ObjectDeduplicator(radius).merge_detection(
                    class_name, point, vector, seen_at_ns, clusters)
                if created:
                    cur.execute('''INSERT INTO semantic_objects (class_name,map_x,map_y,map_z,
                        observation_count,mean_embedding,first_seen_ns,last_seen_ns)
                        VALUES (%s,%s,%s,%s,1,%s,%s,%s) RETURNING object_id''',
                        (class_name,*point.tolist(),vector,int(seen_at_ns),int(seen_at_ns)))
                    object_id = cur.fetchone()[0]
                else:
                    object_id = candidates[next(i for i,c in enumerate(clusters) if c is merged)][0]
                    cur.execute('''UPDATE semantic_objects SET map_x=%s,map_y=%s,map_z=%s,
                        observation_count=%s,mean_embedding=%s,first_seen_ns=%s,last_seen_ns=%s
                        WHERE object_id=%s''', (*merged.map_position.tolist(),merged.observation_count,
                        _vector_value(merged.embedding),merged.first_seen_ns,merged.last_seen_ns,object_id))
            observation_id = self.insert_observation(object_id, seen_at_ns, *robot_pose,
                *camera_pose, bearing_rad, range_m, bbox, confidence, vector, commit=False)
            self.conn.commit()
            return int(object_id), int(observation_id)
        except Exception:
            self.conn.rollback()
            raise


def connect_database(schema='public', **kwargs):
    """Create an explicit schema; never truncate or replace existing data."""
    import psycopg2
    from psycopg2 import sql
    conn = psycopg2.connect(**kwargs)
    with conn.cursor() as cur:
        cur.execute('CREATE EXTENSION IF NOT EXISTS vector WITH SCHEMA public')
        cur.execute(sql.SQL('CREATE SCHEMA IF NOT EXISTS {}').format(sql.Identifier(schema)))
        cur.execute(sql.SQL('SET search_path TO {}, public').format(sql.Identifier(schema)))
    conn.commit()
    init_database(conn)
    return conn
