# .bash-floor/floor.mk -- every recipe of the including Makefile, every `bash`
# a recipe starts and every `#!/usr/bin/env bash` child runs under ONE bash
# >= 4.4, or make stops before any recipe. bash-floor v1
# (reports/deploy-scripts-bash32-silent-exit/).
#
# Why: macOS /bin/bash 3.2.57 ends every fatal shell error of a `set -u` script
# that has an EXIT trap with status 0, so a recipe that only reads exit
# statuses read a crashed smoke as green. The suite therefore requires
# bash >= 4.4.
#
# Use: in a Makefile, before any rule, any other include, any `$(shell ...)`
# (or `!=`, or backtick) and any SHELL assignment -- nothing may run a shell
# before this file has chosen it. Only plain variable definitions may precede
# it, which is where a Makefile that names its own directory with
# $(lastword $(MAKEFILE_LIST)) computes it (after this include, the last word
# is this file):
#
#   include $(dir $(lastword $(MAKEFILE_LIST)))<path to the repo root>/.bash-floor/floor.mk
#
# A Makefile does not assign SHELL itself afterwards; one that wraps the shell
# (a guard around every recipe line) names the floor's bash in it:
# `SHELL := <wrapper> $(WS_BASH)`. The workspace's bash-floor-accounting gate
# checks both, in every Makefile of the suite.
#
# Byte-identical in every repository (bash-floor-parity).
WS_BASH_FLOOR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
# The one resolution (.bash-floor/resolve.sh): WS_BASH as declared (environment
# or command line), else the platform's one candidate. Its refusal is printed
# on stderr and names what to declare.
WS_BASH_FLOOR_RESOLVED := $(shell WS_BASH='$(WS_BASH)' /bin/sh '$(WS_BASH_FLOOR)/resolve.sh')
ifeq ($(WS_BASH_FLOOR_RESOLVED),)
$(error bash-floor: no bash >= 4.4 for this Makefile -- the refusal above names what to declare (reports/deploy-scripts-bash32-silent-exit/))
endif
override WS_BASH := $(WS_BASH_FLOOR_RESOLVED)
export WS_BASH
SHELL := $(WS_BASH)
# Only the floor's shim directory (it holds `bash` and nothing else) goes first:
# WS_BASH's own directory would also put Homebrew's openssl, rsync and every
# other tool it installs ahead of the system's under every recipe.
export PATH := $(WS_BASH_FLOOR)/bin:$(PATH)

.PHONY: bash-floor-canary bash-floor-refusal-probe
# Proves, for the Makefile that includes this file: the recipe shell, a
# recipe's `bash -c` and a child `#!/usr/bin/env bash` script all run bash
# >= 4.4; a script that crashes under `set -u` with an EXIT trap exits non-zero;
# and an absent or old bash is refused before any recipe runs.
bash-floor-canary:
	@WS_BASH_FLOOR_MAKE='$(MAKE)' '$(WS_BASH_FLOOR)/canary.sh' "$$BASH_VERSION"
# The target the canary's refusal cases ask a child make for. It must never
# run: a refused bash stops make while it is still reading this file.
bash-floor-refusal-probe:
	@echo 'bash-floor-refusal-probe: RAN (the floor did not refuse)'
# These targets are defined first; they must not become the default goal.
ifeq ($(.DEFAULT_GOAL),bash-floor-canary)
.DEFAULT_GOAL :=
endif
