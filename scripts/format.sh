#!/usr/bin/env bash
set -e

echo "Running clang-format..."

find src include \
  \( -name "*.cpp" -o -name "*.hpp" -o -name "*.cc" -o -name "*.h" \) \
  -exec clang-format -i {} +

echo "clang-format done."
