#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "${WORKSPACE_DIR}"

# Compatibility wrapper. The legacy monolithic FRD smoke was retired; all
# synthetic decision acceptance now exercises the typed four-node FLU chain.
bash src/falcon_intercept_control/test/run_real_flight_decision_smoke.sh
