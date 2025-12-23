# syntax=docker/dockerfile:1
FROM python:3.10-slim

ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends libgl1 && rm -rf /var/lib/apt/lists/*

COPY python/pyproject.toml /app/pyproject.toml
COPY python/README.md /app/README.md
COPY python/tracker_pkg /app/tracker_pkg
COPY python/scripts /app/tracker_pkg/scripts

RUN pip install --no-cache-dir .[dev]

CMD ["python", "-m", "tracker_pkg.scripts.offline_processor", "--help"]
