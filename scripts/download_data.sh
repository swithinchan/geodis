#!/bin/bash
# Download 3D Pedestrian Network data from HK CSDI Portal
# Requires: curl

set -euo pipefail

DATA_DIR="$(cd "$(dirname "$0")/../data" && pwd)"
COOKIE_JAR="$DATA_DIR/.cookies.txt"

echo "=== Step 1: Visiting CSDI portal to get session ==="
curl -sL -c "$COOKIE_JAR" \
  -H "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36" \
  "https://portal.csdi.gov.hk/geoportal/?datasetId=landsd_rcd_1637222018065_52265&lang=en" \
  -o /dev/null -w "  HTTP %{http_code}\n"

echo "Cookies obtained:"
cat "$COOKIE_JAR"

echo ""
echo "=== Step 2: Downloading JSON format ==="
curl -L -b "$COOKIE_JAR" \
  -H "User-Agent: Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36" \
  -H "Referer: https://portal.csdi.gov.hk/geoportal/?datasetId=landsd_rcd_1637222018065_52265&lang=en" \
  -H "Origin: https://portal.csdi.gov.hk" \
  -o "$DATA_DIR/pedestrian_network.json" \
  "https://portal.csdi.gov.hk/csdi-webpage/download/common/0a5fcd34df9d37bf4309e6b937ebda5355b8fa52eeccbfecd5744a52efe509c7" \
  -w "  HTTP %{http_code}, Size: %{size_download} bytes\n"

echo ""
echo "=== Step 3: Checking result ==="
file "$DATA_DIR/pedestrian_network.json"
ls -lh "$DATA_DIR/pedestrian_network.json"
echo ""
echo "First 300 chars:"
head -c 300 "$DATA_DIR/pedestrian_network.json"
