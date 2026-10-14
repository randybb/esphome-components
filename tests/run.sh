#!/bin/sh
# Compiles the component tests as ESPHome CI does: tests/run.sh [component ...] (default: all)
# ESPHOME=/path/to/esphome to use another installation, MODE=config to only validate.
set -e
cd "$(dirname "$0")"
esphome=${ESPHOME:-esphome}
for component in ${*:-$(ls components)}; do
  for test in components/"$component"/test.*.yaml; do
    platform=${test##*/test.}
    platform=${platform%.yaml}
    echo "=== $component on $platform"
    "$esphome" -s component_name "$component" -s component_test_file "../$test" \
      "${MODE:-compile}" "test_build_components/build_components_base.$platform.yaml"
  done
done
