"""Save outputs of the four required parameterized queries for a recorded run."""
import argparse
import json
import os
from pathlib import Path
import psycopg2
from psycopg2 import sql
from assignment_object_detection.queries import (nearest_object_query,region_query,
    class_histogram_query,object_observations_query)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--schema',required=True);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    with psycopg2.connect(host=os.getenv('PGHOST','localhost'),port=os.getenv('PGPORT','5436'),
        dbname=os.getenv('PGDATABASE','vectordb'),user=os.getenv('PGUSER','postgres'),
        password=os.getenv('PGPASSWORD','postgres')) as conn:
        with conn.cursor() as cur:
            cur.execute(sql.SQL('SET search_path TO {}, public').format(sql.Identifier(a.schema)))
            nearest=nearest_object_query(cur,'chair',5,-1)
            oid=nearest[0][0] if nearest else 1
            result=dict(schema=a.schema,nearest=dict(parameters=['chair',5,-1],rows=nearest),
                region=dict(parameters=[5,9,-4,3],rows=region_query(cur,5,9,-4,3)),
                histogram=class_histogram_query(cur),
                observations=dict(object_id=oid,rows=object_observations_query(cur,oid)))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2))
    print(f'Wrote {a.output}')

if __name__=='__main__':main()
