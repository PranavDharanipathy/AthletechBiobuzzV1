#!/usr/bin/env bash
# Compiles the pure-Java shooter core + simulators + tests with plain javac (no Android, no dependencies)
# and runs the verification suite. Usage: tools/run_tests.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
CORE="$ROOT/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooter/core"
TEST="$ROOT/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/shooter"
javac --release 11 -Xlint:all -Xlint:-options -d "$OUT" "$CORE"/*.java "$TEST"/sim/*.java "$TEST"/*.java
java -cp "$OUT" org.firstinspires.ftc.teamcode.shooter.ShooterTests
