#!/bin/bash
# Adds n building incharge agents to agents.py
# by running ./this_script.sh n
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
FILE_PATH="$PARENT_DIR/agent.py"
n=$1
if [ $n -le 0 ]; then
    exit 1
fi

echo from tools.building_incharge_tools import replyToCI >>"$FILE_PATH"
for i in $(seq 1 $n); do
    cat <<EOT >>"$FILE_PATH"

building_incharge_$i = Agent(
    role='Building Incharge',
    goal='To facilitate visitors to meet their intended host inside their building of care.',
    tools=[replyToCI()],
    memory=True,
    verbose=True
)

EOT
done
echo "$n Buildin Incharge Agent code has been added to ../agent.py."
