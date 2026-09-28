#!/bin/sh
# .bash-floor/resolve.sh -- print the ONE bash this tree runs its shell under,
# or refuse. bash-floor v1 (reports/deploy-scripts-bash32-silent-exit/).
#
# macOS ships /bin/bash 3.2.57, and 3.2 ends every fatal shell error of a
# `set -u` script that has an EXIT trap with status 0: a smoke that crashed half
# way read as green. The suite therefore requires bash >= 4.4 (operator ruling
# 2026-09-27). This file is the rule, and it yields exactly one candidate, which
# is then accepted or refused -- it never searches several bashes by version:
#
#   1. WS_BASH declared   -> that path. It must be absolute, executable and a
#                            bash >= 4.4.
#   2. undeclared, Darwin -> Homebrew's bash under Homebrew's documented default
#                            prefix for the machine: /opt/homebrew on arm64,
#                            /usr/local on x86_64 (the rule Homebrew's installer
#                            itself applies to `uname -m`). Not `brew --prefix`:
#                            brew is often not on PATH where make and agents run,
#                            and a second way to find it would be a fallback. A
#                            non-default prefix is declared with WS_BASH.
#   3. undeclared, other  -> the `bash` on PATH (CI runners and robots: Linux,
#                            bash 5).
#
# Used by .bash-floor/floor.mk (once, when make reads a Makefile) and by
# .bash-floor/bin/bash (when WS_BASH has not been resolved yet). Byte-identical
# in every repository of the suite; the workspace's bash-floor-parity gate
# compares the copies. POSIX sh on purpose: it is what runs BEFORE a good bash
# is known.
set -u

refuse() {
  echo "bash-floor: refused: $1. Declare WS_BASH=<absolute path of a bash >= 4.4> (macOS: brew install bash). See reports/deploy-scripts-bash32-silent-exit/" >&2
  exit 1
}

candidate="${WS_BASH:-}"
if [ -n "$candidate" ]; then
  source_of="WS_BASH (declared)"
else
  system="$(uname -s)" || refuse "uname -s failed"
  case "$system" in
    Darwin)
      machine="$(uname -m)" || refuse "uname -m failed"
      case "$machine" in
        arm64) candidate=/opt/homebrew/bin/bash ;;
        x86_64) candidate=/usr/local/bin/bash ;;
        *) refuse "no Homebrew default prefix is known for Darwin/$machine" ;;
      esac
      source_of="Homebrew's bash for Darwin/$machine (WS_BASH undeclared)"
      ;;
    *)
      candidate="$(command -v bash || true)"
      [ -n "$candidate" ] || refuse "WS_BASH is undeclared and there is no bash on PATH ($system)"
      source_of="the bash on PATH for $system (WS_BASH undeclared)"
      ;;
  esac
fi

case "$candidate" in
  /*) ;;
  *) refuse "$source_of is '$candidate', which is not an absolute path" ;;
esac
case "$candidate" in
  */.bash-floor/bin/bash) refuse "$source_of is '$candidate', the floor's own PATH shim, not a bash" ;;
esac
[ -f "$candidate" ] || refuse "$source_of is '$candidate', which does not exist"
[ -x "$candidate" ] || refuse "$source_of is '$candidate', which is not executable"

# shellcheck disable=SC2016 # expanded by the candidate, not here
version="$("$candidate" -c 'printf "%s %s" "${BASH_VERSINFO[0]}" "${BASH_VERSINFO[1]}"' 2>/dev/null)" \
  || refuse "$source_of is '$candidate', which did not answer as a bash"
major="${version%% *}"
minor="${version#* }"
case "$major$minor" in
  '' | *[!0-9]*) refuse "$source_of is '$candidate', which did not answer as a bash (got '$version')" ;;
esac
if [ "$major" -lt 4 ] || { [ "$major" -eq 4 ] && [ "$minor" -lt 4 ]; }; then
  refuse "$source_of is '$candidate', which is bash $major.$minor (< 4.4)"
fi

printf '%s\n' "$candidate"
