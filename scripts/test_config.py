import struct

Motors = {
    'NotConfigured': {'driver': 'NotConfigured', 'config': {}},
    'RevvyMotor_old':    {
        'driver': 'DcMotor',
        'config': {
            'speed_controller':    [1 / 35, 0.25, 0, -100, 100],
            'position_controller': [4, 0, 0, -600, 600],
            'acceleration_limits': [14400, 3600],
            'encoder_resolution':  1536
        }
    },
    'RevvyMotor': {
        'driver': 'DcMotor',
        'config': {
            'speed_controller': [0.6065, 0.3935, 0, -150, 150],
            'position_controller': [0.1, 0.0000, 0, -150, 150],
            'acceleration_limits': [500, 500],
            'max_current': 1.5,
            'linearity': {0.5: 0, 5.0154: 18, 37.0370: 60, 67.7083: 100, 97.4151: 140, 144.0972: 200},
            'encoder_resolution': 12,
            'gear_ratio': 64.8
        }
    }
}

port_config = Motors["RevvyMotor"]["config"]

resolution = port_config['encoder_resolution'] * port_config['gear_ratio']
(posP, posI, posD, speedLowerLimit, speedUpperLimit) = port_config['position_controller']
(speedP, speedI, speedD, powerLowerLimit, powerUpperLimit) = port_config['speed_controller']
(decMax, accMax) = port_config['acceleration_limits']
max_current = port_config['max_current']
linearity = port_config['linearity']

'''
config = []
config += list(struct.pack("<h", port_config['encoder_resolution']))
config += list(struct.pack("<{}".format("f" * 5), posP, posI, posD, speedLowerLimit, speedUpperLimit))
config += list(struct.pack("<{}".format("f" * 5), speedP, speedI, speedD, powerLowerLimit, powerUpperLimit))
config += list(struct.pack("<ff", decMax, accMax))
'''
config = [
            *struct.pack("<f", resolution),
            *struct.pack("<5f", posP, posI, posD, speedLowerLimit, speedUpperLimit),
            *struct.pack("<5f", speedP, speedI, speedD, powerLowerLimit, powerUpperLimit),
            *struct.pack("<ff", decMax, accMax),
            *struct.pack("<f", max_current)
        ]
for x, y in linearity.items():
            config += struct.pack('<ff', x, y)

print(config)
