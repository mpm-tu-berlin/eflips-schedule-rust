#!/bin/bash
# Stop: block finishing while formatting or tests fail on uncommitted changes. Missing tools are skipped.
input=$(cat)
[ "$(jq -r '.stop_hook_active' <<<"$input")" = "true" ] && exit 0
cd "$CLAUDE_PROJECT_DIR" || exit 0

changed=$(git status --porcelain | awk '{print $NF}')
fail=""
if grep -qE '\.rs$|Cargo\.(toml|lock)$' <<<"$changed" && command -v cargo >/dev/null; then
  out=$(cargo fmt --check 2>&1) || fail+=$'cargo fmt --check failed:\n'"$(tail -20 <<<"$out")"$'\n'
  out=$(cargo test -q 2>&1) || fail+=$'cargo test failed:\n'"$(tail -40 <<<"$out")"$'\n'
fi
if grep -qE '\.py$' <<<"$changed" && command -v black >/dev/null; then
  out=$(black --check test 2>&1) || fail+=$'black --check failed:\n'"$out"$'\n'
fi

[ -n "$fail" ] && jq -n --arg r $'Checks failed. Fix the code, not the tests:\n'"$fail" '{decision: "block", reason: $r}'
exit 0
