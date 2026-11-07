#!/usr/bin/env python3
"""Compile actual route geometry against cached Pedro 3 and check footprint/range assumptions."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--core-jar', type=Path, help='Pedro core 3.0.2 jar; otherwise use Gradle cache')
args = parser.parse_args()
jar = args.core_jar
if jar is None:
    cache = Path(os.environ.get('GRADLE_USER_HOME', str(Path.home() / '.gradle')))
    jars = sorted((cache / 'caches/modules-2/files-2.1/com.pedropathing/core/3.0.2').glob('*/core-3.0.2.jar'))
    if not jars:
        parser.error('Pedro 3.0.2 is not cached; build TeamCode first or supply --core-jar.')
    jar = jars[0]
base = root / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
sources = [root / 'tools/AutoPathGeometryCheck.java',
           root / 'Sim/src/ftcStubs/java/com/acmerobotics/dashboard/config/Config.java',
           base / 'Autonomous/Paths/AutoPathRoutes.java',
           base / 'Constants/AutoPathConstants.java',
           base / 'Constants/FieldConstants.java',
           base / 'Constants/ShooterConstants.java']
with tempfile.TemporaryDirectory(prefix='athletech-auto-geometry-') as output:
    subprocess.run(['javac', '--release', '8', '-Xlint:-options', '-cp', str(jar), '-d', output,
                    *map(str, sources)], check=True)
    subprocess.run(['java', '-cp', output + os.pathsep + str(jar), 'AutoPathGeometryCheck'], check=True)
