#!/usr/bin/env bash
set -euo pipefail

workspace="/mnt/c/Users/saanvi/Documents/Codex/2026-05-10-build-what-this-thing-is-suggesting"
output_dir="$workspace/artifacts/texbat"

mkdir -p "$output_dir"

download() {
  local url="$1"
  local stem="$2"
  echo "Downloading $url"
  curl -L "$url" -o "$output_dir/${stem}_navsol.mat"
}

download "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/cleanStatic/navsol.mat" "cleanStatic"
download "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/ds2/navsol.mat" "ds2"
download "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/ds3/navsol.mat" "ds3"
download "https://rnl-data.ae.utexas.edu/datastore/texbat/processed/ds7/navsol.mat" "ds7"

ls -l "$output_dir"
