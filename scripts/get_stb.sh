#!/usr/bin/env bash
# Downloads stb_image.h and stb_image_write.h into the sim/ directory.
# Run once before building: bash sim/get_stb.sh
set -e
BASE="https://raw.githubusercontent.com/nothings/stb/master"
DIR="$(dirname "$0")/../sim"

curl -sSL "$BASE/stb_image.h"       -o "$DIR/stb_image.h"       && echo "OK: stb_image.h"
curl -sSL "$BASE/stb_image_write.h" -o "$DIR/stb_image_write.h" && echo "OK: stb_image_write.h"
