# so101-lab — convenience targets. Bare `make` prints this help.
# Pass extra flags with ARGS, e.g.  make vr-teleop ARGS="--pos-scale 0.5"
.DEFAULT_GOAL := help
.PHONY: help setup vr-teleop vr-teleop-mock vr-teleop-smoke vr-teleop-hw

ARGS ?=

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'

setup: ## One-shot env setup (uv sync + placo + URDF/MJCF)
	./setup.sh

vr-teleop: ## Real Quest drives the SIM + Rerun plots (put the headset on)
	uv run python vr_teleop.py --source oculus --backend sim --rerun $(ARGS)

vr-teleop-mock: ## No headset: mock controllers drive both sim arms in the MuJoCo viewer
	uv run python vr_teleop.py --view $(ARGS)

vr-teleop-smoke: ## Headless: 240 mock ticks, print per-tick latency + EE travel
	MUJOCO_GL=egl uv run python vr_teleop.py --iters 240 $(ARGS)

vr-teleop-hw: ## Real Quest drives two real SOFollowers (fill ARM_TABLE in multiarm.py first)
	uv run python vr_teleop.py --source oculus --backend hardware $(ARGS)
