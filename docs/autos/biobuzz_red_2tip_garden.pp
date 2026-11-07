{
  "version": "2.0.0",
  "startPoint": {
    "x": 59.25,
    "y": 9.72,
    "headingDeg": 90,
    "locked": false
  },
  "lines": [
    {
      "id": "line-bb-1",
      "color": "#dc2626",
      "name": "To garden",
      "locked": false,
      "waitBeforeMs": 3500,
      "waitAfterMs": 0,
      "waitBeforeName": "Shoot 4 preload: TIP 1 (FAR cell)",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 8.64,
        "y": 22
      },
      "controlPoints": [
        {
          "x": 34,
          "y": 34
        },
        {
          "x": 34,
          "y": 39
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": -90
      }
    },
    {
      "id": "line-bb-2",
      "color": "#f97316",
      "name": "Garden sweep (intake only, FIArm up)",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 600,
      "waitBeforeName": "",
      "waitAfterName": "Intake garden POLLEN",
      "kind": "atomic",
      "endPoint": {
        "x": 8.64,
        "y": 10.72
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": -90
      }
    },
    {
      "id": "line-bb-3",
      "color": "#dc2626",
      "name": "To loading zone",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 3000,
      "waitBeforeName": "",
      "waitAfterName": "Shoot 4: TIP 2 (CLOSE cell), parked",
      "kind": "atomic",
      "endPoint": {
        "x": 15,
        "y": 108
      },
      "controlPoints": [
        {
          "x": 26,
          "y": 30
        },
        {
          "x": 26,
          "y": 90
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": 0
      }
    }
  ],
  "shapes": [
    {
      "id": "biobuzz-red-hive",
      "name": "Red HIVE",
      "vertices": [
        {
          "x": 46,
          "y": 50
        },
        {
          "x": 48.6,
          "y": 50
        },
        {
          "x": 48.6,
          "y": 91.4
        },
        {
          "x": 46,
          "y": 91.4
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "biobuzz-blue-hive",
      "name": "Blue HIVE",
      "vertices": [
        {
          "x": 96.5,
          "y": 91.4
        },
        {
          "x": 93.9,
          "y": 91.4
        },
        {
          "x": 93.9,
          "y": 50
        },
        {
          "x": 96.5,
          "y": 50
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    },
    {
      "id": "biobuzz-flower-top",
      "name": "Scoring-Side FLOWER",
      "vertices": [
        {
          "x": 43.4,
          "y": 141.5
        },
        {
          "x": 50.1,
          "y": 141.5
        },
        {
          "x": 50.1,
          "y": 139.4
        },
        {
          "x": 46.75,
          "y": 137.7
        },
        {
          "x": 43.4,
          "y": 139.4
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "biobuzz-flower-right",
      "name": "Blue Alliance FLOWER",
      "vertices": [
        {
          "x": 141.5,
          "y": 98.1
        },
        {
          "x": 139.4,
          "y": 98.1
        },
        {
          "x": 137.7,
          "y": 94.75
        },
        {
          "x": 139.4,
          "y": 91.4
        },
        {
          "x": 141.5,
          "y": 91.4
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    },
    {
      "id": "biobuzz-flower-bottom",
      "name": "Audience-Side FLOWER",
      "vertices": [
        {
          "x": 91.15,
          "y": 0
        },
        {
          "x": 97.8,
          "y": 0
        },
        {
          "x": 97.8,
          "y": 2.1
        },
        {
          "x": 94.5,
          "y": 3.8
        },
        {
          "x": 91.15,
          "y": 2.1
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    },
    {
      "id": "biobuzz-flower-left",
      "name": "Red Alliance FLOWER",
      "vertices": [
        {
          "x": 0,
          "y": 50.35
        },
        {
          "x": 2.1,
          "y": 50.35
        },
        {
          "x": 3.8,
          "y": 47
        },
        {
          "x": 2.1,
          "y": 43.65
        },
        {
          "x": 0,
          "y": 43.65
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-bb-1"
    },
    {
      "kind": "path",
      "lineId": "line-bb-2"
    },
    {
      "kind": "path",
      "lineId": "line-bb-3"
    }
  ],
  "pathChains": [],
  "fieldPoints": [],
  "robot": {
    "forwardVelocity": 75,
    "strafeVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rLength": 19.44,
    "rWidth": 15.28,
    "safetyMargin": 1,
    "maxAcceleration": 30,
    "maxDeceleration": 30
  },
  "settings": {
    "forwardVelocity": 75,
    "strafeVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rLength": 19.44,
    "rWidth": 15.28,
    "safetyMargin": 1,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "biobuzz.webp"
  }
}
