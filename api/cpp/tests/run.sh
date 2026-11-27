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

# A slide must come back in millimetres, not the metres of its .pos symbol
port=58124
"$server" "$here/../../templates/indexing_table.mstudio" --port $port --speed 20 >/dev/null &
pid2=$!
trap 'kill $pid $pid2 2>/dev/null' EXIT
sleep 1
"$here/build/live_test" $port actuators

# A humanoid walks
port=58125
"$server" --humanoid "Full-size" --port $port --speed 20 >/dev/null &
pid3=$!
trap 'kill $pid $pid2 $pid3 2>/dev/null' EXIT
sleep 1
"$here/build/live_test" $port walking

# A program builds a line in an empty scene and runs it
port=58126
"$server" --port $port --speed 20 >/dev/null &
pid4=$!
trap 'kill $pid $pid2 $pid3 $pid4 2>/dev/null' EXIT
sleep 1
"$here/build/live_test" $port building
