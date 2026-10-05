TABULAR_FIELD_NAMES = {
    "DynamicsSensor": {
        True: [
            # Linear acceleration (m/s²)
            "ax", "ay", "az",
            # Linear velocity (m/s)
            "vx", "vy", "vz",
            # Position (m)
            "px", "py", "pz",
            # Angular acceleration (rad/s²)
            "angular_ax", "angular_ay", "angular_az",
            # Angular velocity (rad/s)
            "angular_vx", "angular_vy", "angular_vz",
            # Orientation (rad)
            "roll", "pitch", "yaw",
        ],
        False: [
            "ax", "ay", "az",
            "vx", "vy", "vz",
            "px", "py", "pz",
            "alphax", "alphay", "alphaz",
            "wx", "wy", "wz",
            "qx", "qy", "qz", "qw",
        ],
    },

    "IMUSensor": {
        False: [
        "ax", "ay", "az",
        "wx", "wy", "wz",
        ],
        True: [
        "ax", "ay", "az",
        "wx", "wy", "wz",
        "bias_ax", "bias_ay", "bias_az",
        "bias_wx", "bias_wy", "bias_wz",
        ],
    },

    # DVLSensor.shape depends on 'ReturnRange' (biguasim/src/biguasim/sensors.py,
    # DVLSensor.__init__): [7] if ReturnRange (default) else [3]. Kept as a
    # dict like IMUSensor/DynamicsSensor so SensorConfig.field_names can pick
    # the matching column set instead of hardcoding 7 (same bug class as the
    # IMU bias bug documented in docs/biguasim_dataset_audit.md section 5).
    "DVLSensor": {
        True: [
            "vx",
            "vy",
            "vz",
            "range_x_front",
            "range_y_front",
            "range_x_back",
            "range_y_back",
        ],
        False: [
            "vx",
            "vy",
            "vz",
        ],
    },

    # DepthSensor (pressure) returns a single scalar [position_z]
    # (biguasim/src/biguasim/sensors.py:1308-1348). Missing from the
    # original biguasim-dataset-main sensor_fields.py -- depth_writer.py was
    # empty there and this sensor was never declared in any of its YAMLs
    # (docs/biguasim_dataset_audit.md section 5), so the gap went unnoticed.
    "DepthSensor": [
        "depth_z",
    ],

    "GPSSensor": [
        "x",
        "y",
        "z",
    ],

    "LocationSensor": [
        "x",
        "y",
        "z",
    ],

    "MagnetometerSensor": [
        "mx",
        "my",
        "mz",
    ],

    "RotationSensor": [
        "roll",
        "pitch",
        "yaw",
    ],

    "OrientationSensor": [
        "forward_x", "right_x", "up_x",
        "forward_y", "right_y", "up_y",
        "forward_z", "right_z", "up_z",
    ],

    "PoseSensor": [
        "r11", "r12", "r13", "px",
        "r21", "r22", "r23", "py",
        "r31", "r32", "r33", "pz",
        "0", "0", "0", "1",
    ],
}