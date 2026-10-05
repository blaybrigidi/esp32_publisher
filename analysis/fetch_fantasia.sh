#!/bin/sh
# Downloads the PhysioNet Fantasia subset used by the HRV studies:
# 10 healthy adults (Y1-Y5 aged 21-34, O1-O5 aged 68-81), ~120 min supine rest,
# one RR interval (s) per line, derived from 250 Hz ECG.
# Iyengar N et al. (1996); Goldberger AL et al., PhysioBank/PhysioNet (2000).
set -e
cd "$(dirname "$0")"
mkdir -p fantasia
for f in Y1 Y2 Y3 Y4 Y5 O1 O2 O3 O4 O5; do
    [ -s "fantasia/$f.txt" ] || curl -sfL -o "fantasia/$f.txt" \
        "https://archive.physionet.org/physiobank/database/fantasia/subset/$f.txt"
done
echo "fantasia/: $(ls fantasia | wc -l | tr -d ' ') files"
