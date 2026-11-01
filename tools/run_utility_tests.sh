#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
PACKAGE=org/firstinspires/ftc/teamcode/Utility
javac --release 8 -Xlint:all -Xlint:-options -d "$OUT" \
  "$ROOT/TeamCode/src/main/java/$PACKAGE/DynamicTrapezoidalSum.java" \
  "$ROOT/TeamCode/src/main/java/$PACKAGE/VoltageCompensator.java" \
  "$ROOT/TeamCode/src/test/java/$PACKAGE/DynamicTrapezoidalSumTest.java" \
  "$ROOT/TeamCode/src/test/java/$PACKAGE/VoltageCompensatorTest.java"
java -cp "$OUT" org.firstinspires.ftc.teamcode.Utility.DynamicTrapezoidalSumTest
java -cp "$OUT" org.firstinspires.ftc.teamcode.Utility.VoltageCompensatorTest
