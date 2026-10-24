#!/usr/bin/env bash
# Run from inside the ns-3.49 folder (MSYS2 UCRT64):   bash ../run_trace.sh 2
# 6 policies x 30 new seeds = 180 short runs. Departure/return times are randomised per seed.

set -u

EXE=$(ls build/scratch/ns3.*-son_trace-* 2>/dev/null | grep -v '\.pdb$' | head -n1)
if [ -z "$EXE" ]; then
  echo "Executable not found. Copy son_trace.cc to scratch/ then run:  python ./ns3 build"
  exit 1
fi
export PATH="$PWD/build/lib:$PATH"

JOBS="${1:-2}"
OUT=../son_trace_runs_180
FAILED=../son_trace_failed.txt
FINAL=../son_trace_results.csv
mkdir -p "$OUT"
: > "$FAILED"

SEEDS=$(seq 4001 4030)
POLICIES="olsr_default olsr_slow h3_k3 h3_k2.5 h3_k2 h5_k2.5"

for s in $SEEDS; do
  for p in $POLICIES; do
    f="$OUT/${p}_${s}.csv"
    [ -s "$f" ] || echo "$EXE --policy=$p --seed=$s --output=$f"
  done
done | xargs -P "$JOBS" -I{} sh -c '{} > /dev/null 2>&1 || echo "FAILED: {}" >> '"$FAILED"

"$EXE" --header=1 --output="$FINAL"
cat "$OUT"/*_*.csv >> "$FINAL"

echo "Data rows written: $(($(wc -l < "$FINAL") - 1))  (8 observers x 180 runs = 1440 expected)"
echo "Failed runs listed in $FAILED:"
cat "$FAILED"
