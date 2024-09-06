#!/bin/bash
# Adds n visitor agents to agents.py
# by running ./this_script.sh n
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
FILE_PATH="$PARENT_DIR/agent.py"
n=$1
if [ $n -le 0 ]; then
    exit 1
fi

echo from tools.visitor_tools import visitorTools >>"$FILE_PATH"
for i in $(seq 1 $n); do
    cat <<EOT >>"$FILE_PATH"

visitor_$i = Agent(
    role='Visitor',
    goal="Meet host at the host's location",
    tools=[visitorTools(result_as_answer=True)],
    memory=True,
    verbose=True
)

EOT
done
echo "$n Visitor Agent code has been added to ../agent.py."
