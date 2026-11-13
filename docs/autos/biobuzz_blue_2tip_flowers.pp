{
  "version": "2.0.0",
  "startPoint": {
    "x": 84.75,
    "y": 134.28,
    "headingDeg": -90,
    "locked": false
  },
  "lines": [
    {
      "id": "line-1",
      "color": "#2563eb",
      "name": "To Blue Alliance FLOWER (shoot first)",
      "locked": false,
      "waitBeforeMs": 3500,
      "waitAfterMs": 0,
      "waitBeforeName": "Shoot 4 preload: TIP 1 (CLOSE cell)",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 122.03,
        "y": 94.75
      },
      "controlPoints": [
        {
          "x": 100,
          "y": 120
        },
        {
          "x": 108,
          "y": 112
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": 0
      }
    },
    {
      "id": "line-2",
      "color": "#16a34a",
      "name": "Touch Blue Alliance FLOWER (dropdown down)",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 1200,
      "waitBeforeName": "",
      "waitAfterName": "Take POLLEN",
      "kind": "atomic",
      "endPoint": {
        "x": 129.01,
        "y": 94.75
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 0,
        "endDeg": 0
      }
    },
    {
      "id": "line-3",
      "color": "#16a34a",
      "name": "Back out",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 122.03,
        "y": 94.75
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 0,
        "endDeg": 0
      }
    },
    {
      "id": "line-4",
      "color": "#2563eb",
      "name": "To Audience-Side FLOWER",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 400,
      "waitBeforeName": "",
      "waitAfterName": "Dropdown down",
      "kind": "atomic",
      "endPoint": {
        "x": 94.475,
        "y": 19.47
      },
      "controlPoints": [
        {
          "x": 116,
          "y": 16
        },
        {
          "x": 108,
          "y": 48
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": 0,
        "endDeg": -90
      }
    },
    {
      "id": "line-4b",
      "color": "#16a34a",
      "name": "In to the shot pose (dropdown down)",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 2150,
      "waitBeforeName": "",
      "waitAfterName": "Shoot alliance FLOWER POLLEN: TIP 2 (FAR cell)",
      "kind": "atomic",
      "endPoint": {
        "x": 94.475,
        "y": 16.99
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": -90
      }
    },
    {
      "id": "line-5",
      "color": "#16a34a",
      "name": "Touch Audience-Side FLOWER (dropdown down)",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 4150,
      "waitBeforeName": "",
      "waitAfterName": "Sit in the FLOWER shooting every ball",
      "kind": "atomic",
      "endPoint": {
        "x": 94.475,
        "y": 10.49
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": -90
      }
    },
    {
      "id": "line-6",
      "color": "#16a34a",
      "name": "Back out",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 94.475,
        "y": 19.47
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": -90
      }
    },
    {
      "id": "line-7",
      "color": "#1e3a8a",
      "name": "Park in loading zone",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 129,
        "y": 36
      },
      "controlPoints": [
        {
          "x": 112,
          "y": 26
        },
        {
          "x": 104,
          "y": 26
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": -90,
        "endDeg": 180
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
      "lineId": "line-1"
    },
    {
      "kind": "path",
      "lineId": "line-2"
    },
    {
      "kind": "path",
      "lineId": "line-3"
    },
    {
      "kind": "path",
      "lineId": "line-4"
    },
    {
      "kind": "path",
      "lineId": "line-4b"
    },
    {
      "kind": "path",
      "lineId": "line-5"
    },
    {
      "kind": "path",
      "lineId": "line-6"
    },
    {
      "kind": "path",
      "lineId": "line-7"
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