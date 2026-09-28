#!/usr/bin/env bash
# .bash-floor/canary.sh -- what `make bash-floor-canary` runs, for the Makefile
# that includes .bash-floor/floor.mk. bash-floor v1
# (reports/deploy-scripts-bash32-silent-exit/).
#
#   canary.sh <the recipe shell's $BASH_VERSION>
#
# Launched directly by the recipe, so its own interpreter IS the proof that a
# child `#!/usr/bin/env bash` script resolves to the floor's bash. It carries no
# prologue on purpose (a prologue would re-exec and hide a PATH that is wrong),
# and no EXIT trap (under bash 3.2 an EXIT trap is exactly what turns a crash
# into status 0): every check records its own verdict and the last line is the
# canary's verdict.
#
# The refusal cases go through the resolver's INPUTS -- WS_BASH declared as an
# absent path, a relative name, the shim itself, stand-ins answering 3.2 and
# 4.3, and the host's /bin/bash when that one is older than 4.4 -- never by
# removing anything from the machine.
floor="$(cd "$(dirname "$0")" && pwd -P)"
make_cmd="${WS_BASH_FLOOR_MAKE:-make}"
recipe_version="${1:-}"
failures=0
refused=()

pass() { echo "  ok    $1"; }
fail() { echo "  FAIL  $1"; failures=$((failures + 1)); }
at_least_44() { # <major> <minor>
  [ "$1" -gt 4 ] || { [ "$1" -eq 4 ] && [ "$2" -ge 4 ]; }
}
version_ok() { # <BASH_VERSION string>
  local major="${1%%.*}" rest="${1#*.}"
  local minor="${rest%%[!0-9]*}"
  case "$major$minor" in '' | *[!0-9]*) return 1 ;; esac
  at_least_44 "$major" "$minor"
}

echo "bash-floor canary ($floor)"

# 1. the recipe shell
if version_ok "$recipe_version"; then pass "recipe shell is bash $recipe_version"; else fail "recipe shell is bash '${recipe_version}' (< 4.4)"; fi
# 2. this script: a child '#!/usr/bin/env bash' launched straight from a recipe
if at_least_44 "${BASH_VERSINFO[0]}" "${BASH_VERSINFO[1]}"; then pass "child #!/usr/bin/env bash script is bash $BASH_VERSION ($BASH)"; else fail "child #!/usr/bin/env bash script is bash $BASH_VERSION ($BASH)"; fi
# 3. a recipe's `bash -c` (the PATH lookup)
bash_c="$(bash -c 'printf %s "$BASH_VERSION"' 2>&1)"
if version_ok "$bash_c"; then pass "bash -c is bash $bash_c ($(command -v bash))"; else fail "bash -c is bash '$bash_c' ($(command -v bash))"; fi

scratch="$(mktemp -d "${TMPDIR:-/tmp}/bash-floor-canary.XXXXXX")" || { echo "bash-floor canary: RED -- mktemp failed"; exit 1; }
# A tree whose .bash-floor is this floor, so a prologue in it finds the shim.
ln -s "$floor" "$scratch/.bash-floor"
{
  echo '#!/usr/bin/env bash'
  cat "$floor/prologue.bash"
  cat <<'PROBE'
set -euo pipefail
trap 'rm -f "$0.never"' EXIT
printf '%s\n' "$BASH_VERSION"
printf '%s\n' "$bash_floor_canary_unbound"
echo reached-the-end
PROBE
} >"$scratch/crash.sh"
{
  echo '#!/usr/bin/env bash'
  cat "$floor/prologue.bash"
  cat <<'PROBE'
set -euo pipefail
printf '%s\n' "$BASH_VERSION"
PROBE
} >"$scratch/version.sh"
chmod +x "$scratch/crash.sh" "$scratch/version.sh"

# 4. the defect itself: a `set -u` crash under an EXIT trap must not exit 0
crash_check() { # <label> <command...>
  local label="$1" out rc
  shift
  out="$("$@" 2>/dev/null)"
  rc=$?
  if [ "$rc" -ne 0 ] && version_ok "${out%%$'\n'*}" && [[ "$out" != *reached-the-end* ]]; then
    pass "$label: a set -u crash under an EXIT trap exits $rc (bash ${out%%$'\n'*})"
  else
    fail "$label: a set -u crash under an EXIT trap exited $rc, output '${out//$'\n'/ | }'"
  fi
}
crash_check "bash crash.sh" bash "$scratch/crash.sh"
crash_check "./crash.sh" "$scratch/crash.sh"

old_bash=""
if [ -x /bin/bash ]; then
  old_version="$(/bin/bash -c 'printf "%s %s" "${BASH_VERSINFO[0]}" "${BASH_VERSINFO[1]}"')"
  if ! at_least_44 "${old_version% *}" "${old_version#* }"; then old_bash=/bin/bash; fi
fi
if [ -n "$old_bash" ]; then
  # 5. the prologue, started by an old bash, re-execs under the floor's bash...
  crash_check "$old_bash crash.sh (prologue re-exec)" env -u WS_BASH "$old_bash" "$scratch/crash.sh"
  out="$(env -u WS_BASH "$old_bash" "$scratch/version.sh" 2>&1)"
  if version_ok "$out"; then pass "$old_bash version.sh re-execs to bash $out"; else fail "$old_bash version.sh answered '$out'"; fi
  # ...and refuses when the declared WS_BASH is absent or is the old bash itself
  for declared in /nonexistent/bash-floor-absent "$old_bash"; do
    out="$(WS_BASH="$declared" "$old_bash" "$scratch/version.sh" 2>&1)"
    rc=$?
    if [ "$rc" -ne 0 ] && [[ "$out" == *bash-floor:\ refused* ]]; then
      pass "prologue refuses WS_BASH=$declared (exit $rc)"
      refused+=("prologue:$declared")
    else
      fail "prologue with WS_BASH=$declared exited $rc: '${out//$'\n'/ | }'"
    fi
  done
else
  echo "  n/a   prologue re-exec from an old bash: this host has no bash < 4.4 at /bin/bash"
fi

# 6. refusal before any recipe, through the resolver's inputs
stand_in() { # <name> <major> <minor>
  printf '#!/bin/sh\nprintf "%%s %%s" %s %s\n' "$2" "$3" >"$scratch/$1"
  chmod +x "$scratch/$1"
  printf '%s\n' "$scratch/$1"
}
old32="$(stand_in bash-3.2 3 2)"
old43="$(stand_in bash-4.3 4 3)"
new44="$(stand_in bash-4.4 4 4)"
make_refuses() { # <how> <WS_BASH value>
  local how="$1" value="$2" out rc
  if [ "$how" = cmdline ]; then
    out="$(env -u MAKEFLAGS -u MFLAGS -u MAKELEVEL -u WS_BASH "$make_cmd" --no-print-directory -f "$floor/floor.mk" "WS_BASH=$value" bash-floor-refusal-probe 2>&1)"
  else
    out="$(env -u MAKEFLAGS -u MFLAGS -u MAKELEVEL WS_BASH="$value" "$make_cmd" --no-print-directory -f "$floor/floor.mk" bash-floor-refusal-probe 2>&1)"
  fi
  rc=$?
  if [ "$rc" -ne 0 ] && [[ "$out" == *"bash-floor: refused"* ]] && [[ "$out" != *"bash-floor-refusal-probe: RAN"* ]]; then
    pass "make refuses WS_BASH=$value ($how, exit $rc, no recipe ran)"
    refused+=("make:$value")
  else
    fail "make with WS_BASH=$value ($how) exited $rc: '${out//$'\n'/ | }'"
  fi
}
make_refuses env /nonexistent/bash-floor-absent
make_refuses cmdline /nonexistent/bash-floor-absent
make_refuses env bash
make_refuses env "$floor/bin/bash"
make_refuses env "$old32"
make_refuses cmdline "$old43"
[ -n "$old_bash" ] && make_refuses env "$old_bash"
# the boundary is inclusive: a bash answering 4.4 is accepted
if [ "$(WS_BASH="$new44" /bin/sh "$floor/resolve.sh" 2>&1)" = "$new44" ]; then pass "resolver accepts a bash answering 4.4"; else fail "resolver does not accept a bash answering 4.4"; fi
if [ "$(env -u WS_BASH /bin/sh "$floor/resolve.sh" 2>&1)" = "$WS_BASH" ]; then pass "undeclared resolution on this host is $WS_BASH, the bash this make resolved"; else echo "  note  undeclared resolution differs from this make's WS_BASH=$WS_BASH (WS_BASH was declared)"; fi

rm -rf "$scratch"
if [ "$failures" -eq 0 ]; then
  echo "bash-floor canary: GREEN -- recipe $recipe_version, bash -c $bash_c, child $BASH_VERSION; refused ${#refused[@]} bad bashes"
  exit 0
fi
echo "bash-floor canary: RED -- $failures check(s) failed"
exit 1
