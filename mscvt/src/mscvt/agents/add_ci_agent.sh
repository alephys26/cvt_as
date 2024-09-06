#!/bin/bash
# Adds n campus incharge agents to agents.py
# by running ./this_script.sh n
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
FILE_PATH="$PARENT_DIR/agent.py"
n=$1
if [ $n -le 0 ]; then
    exit 1
fi

echo from tools.campus_incharge_tools import CITools >>"$FILE_PATH"
for i in $(seq 1 $n); do
    cat <<EOT >>"$FILE_PATH"

campus_incharge_$i = Agent(
    role='Campus Incharge',
    goal="To facilitate visitors to meet their intended host inside the campus from main gate to host's location.",
    tools=[CITools()],
    memory=True,
    verbose=True
)

EOT
done
echo "$n Campus Incharge Agent code has been added to ../agent.py."
