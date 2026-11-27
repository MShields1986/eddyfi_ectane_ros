#!/bin/bash
# Builds the Docker image and runs every test in it, as GitHub Actions does
cd "$(dirname "$0")"
docker compose -f docker/docker-compose.yaml --profile test up --build --remove-orphans --exit-code-from test test
