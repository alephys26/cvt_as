#!/bin/bash
# Adds n find_ci tasks to find_ci.py
# by running ./this_script.sh.n

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
FILE_PATH="$PARENT_DIR/find_ci.py"

if [ $n -le 0 ]; then
    exit 1
fi

echo from tools.visitor_tools import visitorTools >>$FILE_PATH

for i in $(seq 1 $n); do
    cat <<EOT >>"$FILE_PATH"

find_ci_task_$i = Task(
    description='Find an available campus incharge for travelling to the host location',
    agent=visitor$i,
    expected_output='Campus incharge alloted',
    async_execution=false,
    tools=[visitorTools]
)

EOT
done
echo "$n visitor code for finding CI has been added ../find_ci.py."
