#!/usr/bin/env bash
# Stage a self-contained runtime folder for mobile-0-rt (no internet, no pixi env there):
#   <out>/bin/proxy, <out>/lib/<shared libs>, <out>/config/, <out>/models/franka_emika_panda/panda_arm.urdf
# Usage (inside the pixi env): bash scripts/pixi_dist.sh build/proxy dist
set -euo pipefail

binary=$1
out=$2
build_dir=$(dirname "$(realpath "$binary")")

rm -rf "$out"
mkdir -p "$out/bin" "$out/lib" "$out/models/franka_emika_panda"

cp "$binary" "$out/bin/"
# point the binary at ../lib only, so the check below sees exactly what rt will see;
# DT_RPATH (not RUNPATH) also applies to the libs' own deps, e.g. libzerolancom has no rpath
patchelf --force-rpath --set-rpath '$ORIGIN/../lib' "$out/bin/proxy"
# libs from the pixi env and the build dir (zerolancom); glibc and other system libs come from rt itself
ldd "$binary" | awk -v env="$CONDA_PREFIX/" -v build="$build_dir/" \
    'index($3, env) == 1 || index($3, build) == 1 { print $3 }' | while read -r lib; do
    cp -L "$lib" "$out/lib/"
done

cp -r config "$out/"
cp models/franka_emika_panda/panda_arm.urdf "$out/models/franka_emika_panda/"

# fail here instead of on rt if a dependency was not picked up
if ldd "$out/bin/proxy" | grep -q "not found"; then
    ldd "$out/bin/proxy" | grep "not found" >&2
    exit 1
fi
echo "staged $out ($(du -sh "$out" | cut -f1), $(ls "$out/lib" | wc -l) libs)"
