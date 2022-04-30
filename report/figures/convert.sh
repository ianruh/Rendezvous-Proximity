#!/bin/bash

for file in *.svg; do
    baseName=${file::-4}
    inkscape -D -z --export-background="white" --file="$file" --export-pdf="$baseName.pdf" --export-latex
done
