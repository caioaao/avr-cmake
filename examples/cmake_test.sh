#!/usr/bin/env bash

example=$1
out_dir=`mktemp -d`
cmake -S ${example}/src -B ${out_dir}
cd ${out_dir} && make 2>&1 || true # rerouting stderr to stdout to export results
