"""Integration tests use disposable schemas and never touch collected tables."""
import os
import uuid
import pytest
from psycopg2 import sql
from assignment_object_detection.database import connect_database

@pytest.fixture
def conn():
    schema = 'ds685_test_' + uuid.uuid4().hex
    connection = connect_database(schema,host=os.getenv('PGHOST','localhost'),
        port=int(os.getenv('PGPORT','5436')),dbname=os.getenv('PGDATABASE','vectordb'),
        user=os.getenv('PGUSER','postgres'),password=os.getenv('PGPASSWORD','postgres'))
    try:
        yield connection
    finally:
        connection.rollback()
        with connection.cursor() as cur:
            cur.execute(sql.SQL('DROP SCHEMA {} CASCADE').format(sql.Identifier(schema)))
        connection.commit()
        connection.close()
