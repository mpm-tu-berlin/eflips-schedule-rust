#!/bin/bash
# PostToolUse: format the edited file. Missing tools are skipped.
f=$(jq -r '.tool_input.file_path // empty')
case "$f" in
  *.rs) command -v rustfmt >/dev/null && rustfmt --edition 2021 "$f" ;;
  *.py) command -v black >/dev/null && black -q "$f" ;;
esac
exit 0
