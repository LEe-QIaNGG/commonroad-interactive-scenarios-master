#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

replace_suffix_with_seed() {
  local name="$1"
  local seed="$2"
  if [[ -z "$seed" ]]; then
    echo "${name}_generated"
    return
  fi
  if [[ "$name" =~ -([0-9]+)$ ]]; then
    local prefix="${name%-${BASH_REMATCH[1]}}"
    echo "${prefix}-${seed}"
  else
    echo "${name}-${seed}"
  fi
}

INPUT=""
SEED=""
NUM_VEHICLE=""
NUM_EGO=""
MAX_ATTEMPTS=50
OUTPUT_DIR=""
SIM_STEPS=500
STEP_LENGTH=0.1
OVERWRITE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --input) INPUT="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --num-vehicle) NUM_VEHICLE="$2"; shift 2 ;;
    --num-ego) NUM_EGO="$2"; shift 2 ;;
    --max-attempts) MAX_ATTEMPTS="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --simulation-steps) SIM_STEPS="$2"; shift 2 ;;
    --step-length) STEP_LENGTH="$2"; shift 2 ;;
    --overwrite) OVERWRITE=true; shift ;;
    -h|--help)
      echo "请参考 generation/Readme.md 获取 run_full_pipeline.sh 的使用示例。"
      exit 0
      ;;
    *)
      echo "未知参数: $1"
      echo "请查看 generation/Readme.md 获取 run_full_pipeline.sh 的使用示例。"
      exit 1
      ;;
  esac
done

if [[ -z "$INPUT" || -z "$NUM_VEHICLE" || -z "$NUM_EGO" ]]; then
  echo "缺少必填参数 --input/--num-vehicle/--num-ego，详见 generation/Readme.md。" >&2
  exit 1
fi

ABS_INPUT="$(realpath "$INPUT")"
INPUT_DIR="$(dirname "$ABS_INPUT")"
PARENT_BASE="$(basename "$INPUT_DIR")"
PARENT_PARENT="$(dirname "$INPUT_DIR")"

if [[ -n "$OUTPUT_DIR" ]]; then
  TARGET_DIR="$(realpath -m "$OUTPUT_DIR")"
else
  FOLDER_NAME="$(replace_suffix_with_seed "$PARENT_BASE" "$SEED")"
  TARGET_DIR="${PARENT_PARENT}/${FOLDER_NAME}"
fi

INPUT_NAME="$(basename "$ABS_INPUT")"
if [[ "$INPUT_NAME" == *.cr.xml ]]; then
  INPUT_STEM="${INPUT_NAME%.cr.xml}"
else
  INPUT_STEM="$INPUT_NAME"
fi
SCENARIO_NAME="$(replace_suffix_with_seed "$INPUT_STEM" "$SEED")"
SCENARIO_PATH="${TARGET_DIR}/${SCENARIO_NAME}.cr.xml"
SOLUTION_PATH="${TARGET_DIR}/${SCENARIO_NAME}_solution.xml"

OVERWRITE_FLAG=""
if $OVERWRITE; then
  OVERWRITE_FLAG="--overwrite"
fi

INIT_CMD=(
  python "$SCRIPT_DIR/initialize_scenario.py"
  --input "$ABS_INPUT"
  --num-vehicle "$NUM_VEHICLE"
  --num-ego "$NUM_EGO"
  --max-attempts "$MAX_ATTEMPTS"
  --output-dir "$TARGET_DIR"
  $OVERWRITE_FLAG
)

if [[ -n "$SEED" ]]; then
  INIT_CMD+=(--seed "$SEED")
fi

echo ">>> 运行 initialize_scenario.py"
"${INIT_CMD[@]}"

echo ">>> 运行 sumo_config_generator.py"
python "$SCRIPT_DIR/sumo_config_generator.py" \
  --commonroad_file "$SCENARIO_PATH" \
  --output_folder "$TARGET_DIR" \
  --scenario_name "$SCENARIO_NAME" \
  --simulation_steps "$SIM_STEPS" \
  --step_length "$STEP_LENGTH" \
  --solution_file "$SOLUTION_PATH"

echo "✓ 完成：输出目录 $TARGET_DIR"

