#!/bin/bash
# PreToolUse(Bash): refuse `gh pr create` unless the version was bumped vs. origin/main and Cargo.lock matches.
cmd=$(jq -r '.tool_input.command // empty')
[[ "$cmd" == *"gh pr create"* ]] || exit 0
cd "$CLAUDE_PROJECT_DIR" || exit 0

base=$(git merge-base HEAD origin/main 2>/dev/null) || exit 0
version() { grep -m1 '^version' | cut -d'"' -f2; }
old=$(git show "$base:Cargo.toml" | version)
new=$(version <Cargo.toml)
lock=$(grep -A1 '^name = "eflips-schedule-rust"' Cargo.lock | grep '^version' | cut -d'"' -f2)

if [ "$old" = "$new" ]; then
  echo "Cargo.toml version is still $old (same as main). Bump it per SemVer (see CLAUDE.md) before opening the PR." >&2
  exit 2
fi
if [ "$lock" != "$new" ]; then
  echo "Cargo.lock has eflips-schedule-rust $lock but Cargo.toml has $new. Update Cargo.lock to match." >&2
  exit 2
fi
exit 0
