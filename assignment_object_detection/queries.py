"""Query helpers and CLI for the semantic object database."""

from __future__ import annotations

import argparse
import os
from typing import Any, Sequence

import psycopg2
from psycopg2 import sql


def nearest_object_query(cursor: Any, class_name: str, map_x: float, map_y: float) -> list:
    cursor.execute(
        """
        SELECT
            object_id,
            class_name,
            map_x,
            map_y,
            map_z,
            sqrt(power(map_x - %s, 2) + power(map_y - %s, 2)) AS distance
        FROM semantic_objects
        WHERE class_name = %s
        ORDER BY distance
        LIMIT 1
        """,
        (float(map_x), float(map_y), class_name),
    )
    return cursor.fetchall()


def region_query(cursor: Any, x_min: float, x_max: float, y_min: float, y_max: float) -> list:
    cursor.execute(
        """
        SELECT
            object_id,
            class_name,
            map_x,
            map_y,
            map_z,
            observation_count
        FROM semantic_objects
        WHERE map_x BETWEEN %s AND %s
          AND map_y BETWEEN %s AND %s
        ORDER BY class_name, object_id
        """,
        (float(x_min), float(x_max), float(y_min), float(y_max)),
    )
    return cursor.fetchall()


def class_histogram_query(cursor: Any) -> list:
    cursor.execute(
        """
        SELECT
            class_name,
            count(*) AS object_count
        FROM semantic_objects
        GROUP BY class_name
        ORDER BY object_count DESC, class_name
        """
    )
    return cursor.fetchall()


def object_observations_query(cursor: Any, object_id: int) -> list:
    cursor.execute(
        """
        SELECT
            observation_id,
            seen_at_ns,
            map_x,
            map_y,
            map_yaw,
            camera_x,
            camera_y,
            camera_z,
            bearing_rad,
            range_m,
            confidence
        FROM object_observations
        WHERE object_id = %s
        ORDER BY seen_at_ns
        """,
        (int(object_id),),
    )
    return cursor.fetchall()


def build_cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Query the object-detection semantic database.")
    parser.add_argument("--host", default=os.getenv("PGHOST", "localhost"))
    parser.add_argument("--port", type=int, default=int(os.getenv("PGPORT", "5436")))
    parser.add_argument("--dbname", default=os.getenv("PGDATABASE", "vectordb"))
    parser.add_argument("--user", default=os.getenv("PGUSER", "postgres"))
    parser.add_argument("--password", default=os.getenv("PGPASSWORD", "postgres"))
    parser.add_argument("--query", choices=["nearest", "region", "histogram", "observations"], required=True)
    parser.add_argument("--class-name", default="chair")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--x-min", type=float, default=-10.0)
    parser.add_argument("--x-max", type=float, default=10.0)
    parser.add_argument("--y-min", type=float, default=-10.0)
    parser.add_argument("--y-max", type=float, default=10.0)
    parser.add_argument("--object-id", type=int, default=1)
    parser.add_argument('--schema',required=True,help='Recorded run schema')
    return parser


def main() -> None:
    args = build_cli().parse_args()
    conn = psycopg2.connect(
        host=args.host,
        port=args.port,
        dbname=args.dbname,
        user=args.user,
        password=args.password,
    )
    try:
        cursor = conn.cursor()
        cursor.execute(sql.SQL('SET search_path TO {}, public').format(sql.Identifier(args.schema)))
        if args.query == "nearest":
            rows = nearest_object_query(cursor, args.class_name, args.x, args.y)
        elif args.query == "region":
            rows = region_query(cursor, args.x_min, args.x_max, args.y_min, args.y_max)
        elif args.query == "histogram":
            rows = class_histogram_query(cursor)
        else:
            rows = object_observations_query(cursor, args.object_id)

        for row in rows:
            print(row)
    finally:
        conn.close()


if __name__ == "__main__":
    main()
