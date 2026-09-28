# bash-floor v1: bash >= 4.4, or re-exec under the tree's resolved bash, or refuse (reports/deploy-scripts-bash32-silent-exit/).
case "${BASH_VERSION:-}" in [0-3].*|4.[0-3].*)
  _bash_floor="$(cd "$(dirname "$0")" 2>/dev/null && pwd -P)"
  while [ -n "$_bash_floor" ] && [ ! -x "$_bash_floor/.bash-floor/bin/bash" ]; do _bash_floor="${_bash_floor%/*}"; done
  if [ -n "$_bash_floor" ] && [ "${BASH_SOURCE[0]:-}" = "$0" ] && [ -f "$0" ] && [ "${WS_BASH_FLOOR_REEXEC:-}" != "$$" ]; then
    WS_BASH_FLOOR_REEXEC="$$" PATH="$_bash_floor/.bash-floor/bin:$PATH" exec "$_bash_floor/.bash-floor/bin/bash" "$0" "$@"
  fi
  echo "bash-floor: refused: ${BASH_SOURCE[0]:-$0} needs bash >= 4.4 and runs under bash ${BASH_VERSION} (${BASH:-?}); declare WS_BASH=<absolute path of a bash >= 4.4> (macOS: brew install bash)" >&2
  # shellcheck disable=SC2317 # exit is reached when this file is executed, return when it is sourced
  return 1 2>/dev/null || exit 1 ;;
esac
