#!/usr/bin/env bash
# run once: downloads stb_image.h into the sim/ directory
curl -sSL https://raw.githubusercontent.com/nothings/stb/master/stb_image.h \
     -o stb_image.h && echo "stb_image.h downloaded OK"
