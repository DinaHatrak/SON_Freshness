#!/usr/bin/env bash
# Run from inside the ns-3.49 folder, in the MSYS2 UCRT64 shell:
#     bash ../run_all.sh 2        # 2 = number of parallel jobs
# Each run is a separate process (fixes RNG-stream alignment across policies).
# Safe to stop and restart: finished runs are skipped.

set -u

EXE=$(ls build/scratch/ns3.*-son_study2-* 2>/dev/null | grep -v '\.pdb$' | head -n1)
if [ -z "$EXE" ]; then
  echo "Executable not found. Build first:  python ./ns3 build"
  exit 1
fi

export PATH="$PWD/build/lib:$PATH"

JOBS="${1:-2}"
OUT=../son_v2_runs
FAILED=../son_v2_failed.txt
FINAL=../son_ns3_v2_results.csv
mkdir -p "$OUT"
: > "$FAILED"

# New seeds, never used in earlier experiments.
SEEDS="2003 2011 2017 2027 2029 2039 2053 2063 2069 2081"
SCENARIOS="stable fading churn outage stress"
SIZES="25 50 100"
POLICIES="olsr_default olsr_slow h3_k3 h3_k2.5 h3_k2 h5_k2.5"

# Smoke test (must pass before the full run)
SMOKE="$OUT/_smoke.csv"
rm -f "$SMOKE" "$SMOKE.2"
"$EXE" --scenario=churn --policy=olsr_default --nodes=25 --seed=1 --output="$SMOKE" || { echo "Smoke test failed"; exit 1; }
"$EXE" --scenario=churn --policy=olsr_default --nodes=25 --seed=1 --output="$SMOKE.2" || { echo "Smoke test failed"; exit 1; }
if ! diff -q "$SMOKE" "$SMOKE.2" > /dev/null; then
  echo "WARNING: identical runs gave different rows (non-deterministic). Do not continue."
  exit 1
fi
rm -f "$SMOKE" "$SMOKE.2"

for sc in $SCENARIOS; do
  for n in $SIZES; do
    for s in $SEEDS; do
      for p in $POLICIES; do
        f="$OUT/${sc}_${p}_${n}_${s}.csv"
        [ -s "$f" ] || echo "$EXE --scenario=$sc --policy=$p --nodes=$n --seed=$s --output=$f"
      done
    done
  done
done | xargs -P "$JOBS" -I{} sh -c '{} > /dev/null 2>&1 || echo "FAILED: {}" >> '"$FAILED"

"$EXE" --header=1 --output="$FINAL"
cat "$OUT"/*_*_*_*.csv >> "$FINAL"

echo "Rows written (should be 900): $(($(wc -l < "$FINAL") - 1))"
echo "Failed runs listed in $FAILED:"
cat "$FAILED"
