#!/usr/bin/env python3
"""Small VTD RDB-compatible server used for bridge integration smoke tests."""

import argparse
import math
import socket
import struct
import threading
import time


RDB_MAGIC = 35712
RDB_VERSION = 0x0125
RDB_PKG_ID_END_OF_FRAME = 2
RDB_PKG_ID_OBJECT_STATE = 9
RDB_PKG_ID_DRIVER_CTRL = 26
RDB_PKG_FLAG_EXTENDED = 1
RDB_COORD_TYPE_INERTIAL = 0
RDB_COORD_TYPE_PLAYER = 2


def coord(x, y, z, heading, pitch, roll, coord_type):
    return struct.pack(
        "<dddfffBBH", x, y, z, heading, pitch, roll, 3, coord_type, 0
    )


def object_state(frame, sim_time):
    name = b"Ego" + bytes(29)
    base = struct.pack("<IBBH32s", 1, 1, 1, 0, name)
    base += struct.pack("<ffffff", 4.8, 1.9, 1.5, 1.2, 0.0, 0.75)
    base += coord(10.0 + sim_time * 5.0, 20.0, 0.0, 0.1, 0.0, 0.0, RDB_COORD_TYPE_INERTIAL)
    base += struct.pack("<IHh", 0, 0, 0)
    extension = coord(5.0, 0.1, 0.0, 0.02, 0.0, 0.0, RDB_COORD_TYPE_PLAYER)
    extension += coord(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, RDB_COORD_TYPE_PLAYER)
    extension += struct.pack("<fIII", sim_time * 5.0, 0, 0, 0)
    payload = base + extension
    assert len(payload) == 208
    object_entry = struct.pack(
        "<IIIHH", 16, len(payload), len(payload), RDB_PKG_ID_OBJECT_STATE, RDB_PKG_FLAG_EXTENDED
    ) + payload
    end_entry = struct.pack("<IIIHH", 16, 0, 0, RDB_PKG_ID_END_OF_FRAME, 0)
    data = object_entry + end_entry
    header = struct.pack(
        "<HHIIId", RDB_MAGIC, RDB_VERSION, 24, len(data), frame, sim_time
    )
    return header + data


def receive_controls(connection, stop):
    buffered = bytearray()
    connection.settimeout(0.2)
    while not stop.is_set():
        try:
            chunk = connection.recv(65536)
        except socket.timeout:
            continue
        except OSError:
            return
        if not chunk:
            return
        buffered.extend(chunk)
        while len(buffered) >= 24:
            magic, _version, header_size, data_size, frame, sim_time = struct.unpack_from(
                "<HHIIId", buffered
            )
            if magic != RDB_MAGIC:
                del buffered[0]
                continue
            total = header_size + data_size
            if len(buffered) < total:
                break
            offset = header_size
            while offset + 16 <= total:
                entry_header, entry_data, element_size, package_id, _flags = struct.unpack_from(
                    "<IIIHH", buffered, offset
                )
                if package_id == RDB_PKG_ID_DRIVER_CTRL and entry_data >= 32:
                    player = struct.unpack_from("<I", buffered, offset + entry_header)[0]
                    acceleration = struct.unpack_from("<f", buffered, offset + entry_header + 24)[0]
                    steering = struct.unpack_from("<f", buffered, offset + entry_header + 28)[0]
                    print(
                        f"CONTROL frame={frame} time={sim_time:.3f} "
                        f"player={player} accel={acceleration:.3f} steer={steering:.3f}",
                        flush=True,
                    )
                offset += entry_header + entry_data
                if element_size == 0 and entry_data == 0:
                    continue
            del buffered[:total]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=49090)
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--period", type=float, default=0.05)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(1)
        print(f"LISTENING {args.host}:{args.port}", flush=True)
        connection, address = server.accept()
        print(f"CONNECTED {address[0]}:{address[1]}", flush=True)
        with connection:
            stop = threading.Event()
            receiver = threading.Thread(
                target=receive_controls, args=(connection, stop), daemon=True
            )
            receiver.start()
            try:
                for frame in range(1, args.frames + 1):
                    sim_time = frame * args.period
                    connection.sendall(object_state(frame, sim_time))
                    time.sleep(args.period)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                stop.set()
                receiver.join(timeout=1.0)


if __name__ == "__main__":
    main()
