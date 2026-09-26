# `make sync` is the whole setup on a robot or a workstation: uv builds
# .venv from uv.lock (base + [firmware,dev], see [dependency-groups] in
# pyproject.toml) and the mkit-* commands are linked into $(BIN_DIR), so they
# run from any shell without activating the venv. The links point into this
# checkout's .venv, so a later `git pull && make sync` is all an update takes.

BIN_DIR ?= $(HOME)/.local/bin
CLIS := mkit-teach mkit-urdf mkit-toolconfig mkit-firmware-client

.PHONY: sync lock test unlink

sync:
	uv sync
	mkdir -p $(BIN_DIR)
	for c in $(CLIS); do ln -sfn $(CURDIR)/.venv/bin/$$c $(BIN_DIR)/$$c; done
	@case ":$$PATH:" in *":$(BIN_DIR):"*) ;; \
	  *) echo "NOTE: $(BIN_DIR) is not on PATH — add it to ~/.bashrc";; esac
	@echo "Next: mkit-teach record"

lock:
	uv lock

test:
	uv run pytest -q -rs

unlink:
	for c in $(CLIS); do rm -f $(BIN_DIR)/$$c; done
