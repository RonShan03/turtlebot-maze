# DS685: independent RGB-D object detection and semantic memory

This is the implementation and measured submission for the [ROS object-detection assignment](https://aegean.ai/aiml-common/assignments/topics/ROS/object-detection). Read [REPORT.pdf](REPORT.pdf), [REPORT.html](REPORT.html), or [REPORT.md](REPORT.md) for the experiment, results, screenshots, and limitations. `results/house03/` contains all 225 RGB frames and their annotated counterparts, ten contact sheets, database exports, annotations, query outputs, and the semantic map.

## Run a new experiment

Run commands from the repository root. Prerequisites: Docker with Linux containers and host networking enabled, the course ROS Jazzy/Gazebo images, and internet access for the first YOLO/CLIP model download. On Windows use Docker Desktop host networking; running the Zenoh client directly on the Windows host did not reach the Docker host-network router in this environment.

Build the course images if they are not already installed:

```powershell
docker compose build overlay
docker build -f docker/Dockerfile.torch.gpu -t turtlebot-maze-embedding-ingest .
docker compose -f docker-compose.yaml -f compose.assignment.yaml build assignment-detector
```

Choose a fresh run name each time. Do not reuse an existing output directory or database schema. Start a fresh simulation at the origin for this route (stop/recreate only the house service if it is already running):

```powershell
$env:DS685_RUN = 'myrun01'
$env:DS685_DURATION = '1800'
docker compose -f docker-compose.yaml -f compose.assignment.yaml up -d vector zenoh-router zenoh-bridge demo-world-house
```

The submitted experiment uses Gazebo world-pose localization to isolate perception and association errors. After Nav2 starts, disable AMCL's competing transform and run the localization helper in a second terminal:

```powershell
docker compose exec demo-world-house bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 param set /amcl tf_broadcast false'
docker compose -f docker-compose.yaml -f compose.assignment.yaml exec demo-world-house bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /assignment/scripts/sim_localization.py --output /assignment/runs/localization_myrun01.jsonl'
```

Then start the independent detector and drive the route:

```powershell
docker compose -f docker-compose.yaml -f compose.assignment.yaml up -d --no-deps assignment-detector
docker compose -f docker-compose.yaml -f compose.assignment.yaml logs -f assignment-detector
# In another terminal, after the detector prints RUNNING:
docker compose -f docker-compose.yaml -f compose.assignment.yaml exec demo-world-house bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /assignment/scripts/drive_route.py --initialize --route /assignment/route.json --output /assignment/runs/myrun01/navigation.jsonl --timeout 240'
```

`--initialize` is only appropriate for a fresh robot at (0,0,0). The driver stops on a failed goal and records that outcome. When navigation finishes, finalize the detector metadata gracefully, then Ctrl+C the localization helper:

```powershell
docker compose -f docker-compose.yaml -f compose.assignment.yaml kill -s SIGINT assignment-detector
```

To evaluate ordinary localization, leave AMCL broadcasting and omit the simulation helper; label that as a separate experiment. The independent detector reads `/tf` in either case and refuses odometry-frame poses without a map transform. It imports no `rclpy`; ROS imports occur only in the navigation, localization, and fixture-generation utilities inside the simulation container.

## Tests, exports, queries, and analysis

Install `requirements.txt` in a Python environment. The tested runtime dependency versions are recorded in `results/house03/environment.json`. The integration tests need the vector service on localhost:5436; credentials default to the course's local postgres/postgres, database vectordb. Override with PGHOST, PGPORT, PGDATABASE, PGUSER, PGPASSWORD. Tests create disposable UUID schemas and never truncate collected data.

```powershell
python -m pytest -q assignment_object_detection/tests
python -m assignment_object_detection.scripts.export_database --schema ds685_myrun01 --output-dir assignment_object_detection/runs/myrun01/export
python -m assignment_object_detection.scripts.query_demo --schema ds685_myrun01 --output assignment_object_detection/runs/myrun01/queries.json
python -m assignment_object_detection.queries --schema ds685_myrun01 --query nearest --class-name chair --x 5 --y -1
python -m assignment_object_detection.queries --schema ds685_myrun01 --query region --x-min 5 --x-max 9 --y-min -4 --y-max 3
python -m assignment_object_detection.queries --schema ds685_myrun01 --query histogram
python -m assignment_object_detection.queries --schema ds685_myrun01 --query observations --object-id 1
```

The four SQL helpers bind all user values as parameters. Schema names use psycopg2 identifiers. Both exported tables come from one repeatable-read snapshot. Each accepted detection creates exactly one evidence row in the same transaction as its object update.

Reproduce the submitted analysis without running Gazebo or loading neural networks:

```powershell
python -m assignment_object_detection.scripts.evaluate --run assignment_object_detection/results/house03 --truth assignment_object_detection/ground_truth.csv --annotations assignment_object_detection/results/house03/annotations.json --output assignment_object_detection/results/house03/metrics.csv
python -m assignment_object_detection.scripts.association_radius_eval --run assignment_object_detection/results/house03 --truth assignment_object_detection/ground_truth.csv --annotations assignment_object_detection/results/house03/annotations.json --output assignment_object_detection/results/house03/radius_eval.csv
python -m assignment_object_detection.scripts.noise_analysis --run assignment_object_detection/results/house03 --truth assignment_object_detection/ground_truth.csv --annotations assignment_object_detection/results/house03/annotations.json --output assignment_object_detection/results/house03/noise.csv
python -m assignment_object_detection.scripts.semantic_map_plot --input assignment_object_detection/results/house03/export/semantic_objects.csv --route assignment_object_detection/results/house03/route.jsonl --output assignment_object_detection/results/house03/semantic_map.png
python -m assignment_object_detection.scripts.verify_run --run assignment_object_detection/results/house03 --localization assignment_object_detection/results/house03/localization.jsonl
```

For a new run, create contact sheets with `scripts.contact_sheet --run ...`, inspect every frame, and annotate visible classes plus each detection's physical instance ID or null. The evaluator rejects missing or unreviewed frames. `scripts/review_house03.py` records the submitted run's review and is not an annotator for future runs. Raw depth arrays are archived as `depth_frames.npz`; RGB evidence, embeddings, original observations, and poses are retained for replay. The archive keys are original depth filenames without `.npy`.

To regenerate the PDF and self-contained HTML report, install `requirements-report.txt` and run `python -m assignment_object_detection.scripts.render_report`. The PDF includes six selected annotated screenshots and the semantic map. All images are actual captured simulation evidence.

## Files

| Requirement | Implementation/evidence |
|---|---|
| Independent CDR subscriber and inference | runtime.py, messages.py, synchronization.py, detector_pipeline.py |
| Transform chain and numeric verification | geometry.py, tests/test_geometry.py |
| Schema and transactional association | sql/semantic_schema.sql, database.py, deduplication.py |
| Four parameterized queries | queries.py, results/house03/queries.json |
| Database export | results/house03/export/*.csv |
| Coverage and semantic map | route.json, results/house03/navigation.jsonl, semantic_map.png |
| Ground truth and measured evaluation | ground_truth.csv, annotations.json, metrics.csv, radius_eval.csv, noise.csv |
| Discussion and limitations | REPORT.md |
