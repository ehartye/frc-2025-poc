window.sourceMatch = {
  "pipes": [
    {
      "id": "coral-A",
      "clock": [
        [
          "Human releases",
          3.1
        ],
        [
          "Pipe leaves ramp",
          3.671672635314282
        ],
        [
          "Seat attained",
          4.821483894157266
        ],
        [
          "Chassis in transit",
          8.821483895314282
        ],
        [
          "Placement starts",
          29.79948389531428
        ],
        [
          "First target contact",
          30.05948389531428
        ],
        [
          "Sustained support",
          32.05910670547638
        ]
      ],
      "spin": {
        "-29.690655": "inner-riser-bound",
        "-20": "inner-riser-bound",
        "-15": "inner-riser-bound",
        "-10": "inner-riser-bound",
        "-5": "inner-riser-bound",
        "0": "sustained-paired-floor-and-lip",
        "5": "sustained-paired-floor-and-lip",
        "10": "sustained-paired-floor-and-lip",
        "15": "sustained-paired-floor-and-lip",
        "20": "receiver-recontact",
        "29.690655": "receiver-recontact"
      }
    },
    {
      "id": "coral-B",
      "clock": [
        [
          "Human releases",
          53.501357467326606
        ],
        [
          "Pipe leaves ramp",
          54.073030102640885
        ],
        [
          "Seat attained",
          55.22284136148387
        ],
        [
          "Chassis in transit",
          59.222841362640885
        ],
        [
          "Placement starts",
          79.88784136264088
        ],
        [
          "First target contact",
          80.14784136264089
        ],
        [
          "Sustained support",
          82.1474641729894
        ]
      ],
      "spin": {
        "-29.690655": "inner-riser-bound",
        "-20": "inner-riser-bound",
        "-15": "inner-riser-bound",
        "-10": "inner-riser-bound",
        "-5": "inner-riser-bound",
        "0": "sustained-paired-floor-and-lip",
        "5": "sustained-paired-floor-and-lip",
        "10": "sustained-paired-floor-and-lip",
        "15": "sustained-paired-floor-and-lip",
        "20": "receiver-recontact",
        "29.690655": "receiver-recontact"
      }
    }
  ],
  "scenarios": {
    "centered": {
      "support": [
        {
          "id": "coral-A",
          "support_s": 32.05910670547638,
          "sector_m": -0.18
        },
        {
          "id": "coral-B",
          "support_s": 82.1474641729894,
          "sector_m": 0.22
        }
      ],
      "lip_arrivals": [
        3.671672635314282,
        54.073030102640885
      ],
      "scheduled_end_s": 94.01537625603031,
      "park_overlap_m": 0.7663125000000015,
      "first_body_pair": null,
      "min_interrobot_gap_m": 0.10550422873272979,
      "model_conditional_point_ceiling": 9
    },
    "partner-holds-start": {
      "support": [],
      "lip_arrivals": [
        3.671672635314282,
        54.073030102640885
      ],
      "scheduled_end_s": 94.01537625603031,
      "park_overlap_m": 0.7663125000000015,
      "first_body_pair": [
        "red2",
        "red1",
        89.39,
        -0.00506
      ],
      "min_interrobot_gap_m": -0.0865812065230382,
      "model_conditional_point_ceiling": null
    },
    "return-to-station": {
      "support": [
        {
          "id": "coral-A",
          "support_s": 32.05910670547638,
          "sector_m": -0.18
        },
        {
          "id": "coral-B",
          "support_s": 82.1474641729894,
          "sector_m": 0.22
        }
      ],
      "lip_arrivals": [
        3.671672635314282,
        54.073030102640885
      ],
      "scheduled_end_s": 100.10361194948402,
      "park_overlap_m": 0,
      "first_body_pair": null,
      "min_interrobot_gap_m": 0.10550422873272912,
      "model_conditional_point_ceiling": 7
    },
    "second-stock-late": {
      "support": [
        {
          "id": "coral-A",
          "support_s": 32.05910670547638,
          "sector_m": -0.18
        }
      ],
      "lip_arrivals": [
        3.671672635314282
      ],
      "scheduled_end_s": 52.40135746710645,
      "park_overlap_m": 0,
      "first_body_pair": null,
      "min_interrobot_gap_m": 0.10550422873272979,
      "model_conditional_point_ceiling": 5
    }
  },
  "rebounds": [
    [
      0.2,
      0.035
    ],
    [
      0.4,
      0.02
    ],
    [
      0.6,
      0.015
    ]
  ]
};
