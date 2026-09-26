#!/usr/bin/env python3
"""HK byte/CRC helpers for the PTY simulator; exact legacy wire layout, no ROS fixture publishers."""
import struct

# Calculate the deployed legacy header CRC using its exact nonstandard lookup table.
def crc8(data: bytes) -> int:
    # Exact lookup table copied from the deployed HK legacy contract.  This
    # protocol uses a non-standard CRC-8 table, so a generic polynomial helper
    # would test the wrong wire format.
    table = bytes.fromhex(
        "005ebce2613fdd83c29c7e20a3fd1f419dc3217ffca2401e5f01e3bd3e6082dc"
        "237d9fc1421cfea0e1bf5d0380de3c62bee0025cdf81633d7c22c09e1d43a1ff"
        "4618faa427799bc584da3866e5bb5907db856739bae406581947a5fb7826c49a"
        "653bd987045ab8e6a7f91b45c6987a24f8a6441a99c7257b3a6486d85b05e7b9"
        "8cd2306eedb3510f4e10f2ac2f7193cd114fadf3702ecc92d38d6f31b2ec0e50"
        "aff1134dce90722c6d33d18f0c52b0ee326c8ed0530defb1f0ae4c1291cf2d73"
        "ca947628abf517490856b4ea6937d58b5709ebb536688ad495cb2977f4aa4816"
        "e9b7550b88d6346a2b7597c94a14f6a8742ac896154ba9f7b6e80a54d7896b35")
    value = 0xFF
    for byte in data:
        value = table[value ^ byte]
    return value


# Calculate the deployed whole-packet CRC, excluding the checksum and trailer.
def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0x8408 if value & 1 else value >> 1
    return value & 0xFFFF


# Encode a real HK referee packet; scenario state changes only game progress and robot HP.
def game_frame(progress=0, hp=321) -> bytes:
    header = bytearray(struct.pack("<2sHBBBB", b"HK", 78, 1, 0, 7, 0))
    header.append(crc8(header))
    payload = struct.pack(
        "<4B8H10hBHB2B4H2f3B",
        progress, 2, 7, 1,          # explicit simulated referee state
        100, 200, 300, 0, 400, 500, 600, 1,  # red / blue HP
        123, -456, 20, 30, 40, 50, 60, 70, 80, 90,  # enemy positions in cm
        3, 0x1234, 0, 1, 0,         # radar and revival flags
        hp, 400, 77, 0,             # simulated robot health / ammunition
        1.25, -2.5, 1, 0x21, 0,
    )
    frame = bytes(header) + payload
    return frame + struct.pack("<H", crc16(frame)) + b"KH"
