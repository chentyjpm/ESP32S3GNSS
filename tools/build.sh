#!/usr/bin/env bash
set -euo pipefail

if ! command -v idf.py >/dev/null 2>&1; then
  echo "idf.py not found. Please install ESP-IDF and export the environment (source \${IDF_PATH:-<esp-idf>}/export.sh)."
  exit 127
fi

echo "Building ESP32-S3 GNSS demo..."
idf.py build
