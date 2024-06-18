#!/bin/bash

if [ -z "$1" ]; then
  echo "Usage: $0 <base_name>"
  exit 1
fi
BASE_NAME=$1

for svg_file in ${BASE_NAME}*.svg; do
    png_file="${svg_file%.svg}.png"
    rsvg-convert -o "$png_file" "$svg_file"
done

convert -delay 5 -loop 0 $(ls -v ${BASE_NAME}*.png) ${BASE_NAME}.gif

rm ${BASE_NAME}*.svg
rm ${BASE_NAME}*.png

echo "GIF created successfully"
