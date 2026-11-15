#!/bin/sh
# Builds the library, starts motionstudio-server with a 6-axis robot and runs
# the live test against it. MOTIONSTUDIO_SERVER overrides the server's path.
set -e
here=$(cd "$(dirname "$0")/.." && pwd)
server=${MOTIONSTUDIO_SERVER:-$here/../../build/motionstudio-server}
cmake -S "$here" -B "$here/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$here/build" -j >/dev/null
port=58123
"$server" --robot "6-axis 0.9" --port $port --speed 20 >/dev/null &
pid=$!
trap 'kill $pid 2>/dev/null' EXIT
sleep 1
"$here/build/live_test" $port
