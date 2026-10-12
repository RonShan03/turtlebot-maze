# Reuse the course perception image (already includes YOLO and CLIP weights).
ARG DETECTOR_IMAGE=turtlebot-maze-embedding-ingest
FROM ${DETECTOR_IMAGE}
USER root
RUN pip install --no-cache-dir pgvector matplotlib PyYAML
ENV PYTHONUNBUFFERED=1
WORKDIR /work
