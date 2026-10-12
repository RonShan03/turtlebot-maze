#!/usr/bin/env python3
"""Export semantic object and observation tables as CSV."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path

import psycopg2
from psycopg2 import sql


def main() -> None:
    parser = argparse.ArgumentParser(description="Export semantic database tables")
    parser.add_argument("--host", default=os.getenv("PGHOST", "localhost"))
    parser.add_argument("--port", type=int, default=int(os.getenv("PGPORT", "5436")))
    parser.add_argument("--dbname", default=os.getenv("PGDATABASE", "vectordb"))
    parser.add_argument("--user", default=os.getenv("PGUSER", "postgres"))
    parser.add_argument("--password", default=os.getenv("PGPASSWORD", "postgres"))
    parser.add_argument("--output-dir", default="assignment_object_detection/output")
    parser.add_argument('--schema',required=True)
    args = parser.parse_args()

    conn = psycopg2.connect(host=args.host, port=args.port, dbname=args.dbname, user=args.user, password=args.password)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        with conn.cursor() as cur:
            cur.execute(sql.SQL('SET search_path TO {}, public').format(sql.Identifier(args.schema)))
            cur.execute('SET TRANSACTION ISOLATION LEVEL REPEATABLE READ READ ONLY')
            for table in ["semantic_objects", "object_observations"]:
                cur.execute(sql.SQL('SELECT * FROM {} ORDER BY 1').format(sql.Identifier(table)))
                rows = cur.fetchall()
                columns = [desc[0] for desc in cur.description]
                path = out_dir / f"{table}.csv"
                with path.open("w", newline="", encoding="utf-8") as fh:
                    writer = csv.writer(fh)
                    writer.writerow(columns)
                    writer.writerows(rows)
                print(f"Wrote {path} ({len(rows)} rows)")
    finally:
        conn.close()


if __name__ == "__main__":
    main()
