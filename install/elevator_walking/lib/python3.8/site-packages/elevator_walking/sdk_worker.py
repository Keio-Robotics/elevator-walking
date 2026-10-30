#!/usr/bin/env python3
"""Run Unitree SDK calls in a process that does not load ROS 2 Python."""

import json
import socket
import sys


def _send_message(channel, message):
    channel.sendall((json.dumps(message) + "\n").encode("utf-8"))


def _stop_client(client):
    try:
        return client.StopMove()
    except Exception:
        return None


def main():
    if len(sys.argv) != 3:
        return 2

    channel = socket.socket(fileno=int(sys.argv[1]))
    network_interface = sys.argv[2]
    client = None
    is_moving = False
    reader = None

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        ChannelFactoryInitialize(0, network_interface)
        client = SportClient()
        client.SetTimeout(3.0)
        client.Init()
        _send_message(channel, {"type": "ready"})

        reader = channel.makefile("rb")
        while True:
            line = reader.readline()
            if not line:
                break

            try:
                command = json.loads(line.decode("utf-8"))
                operation = command["operation"]

                if operation == "move":
                    result = client.Move(
                        command["vx"], command["vy"], command["vyaw"]
                    )
                    if result not in (None, 0):
                        _send_message(
                            channel,
                            {
                                "type": "error",
                                "message": f"Move returned error code {result}",
                            },
                        )
                    else:
                        is_moving = True
                elif operation == "stop":
                    result = client.StopMove()
                    is_moving = False
                    _send_message(
                        channel, {"type": "stop_result", "result": result}
                    )
                elif operation == "shutdown":
                    if is_moving:
                        result = client.StopMove()
                        is_moving = False
                        _send_message(
                            channel, {"type": "stop_result", "result": result}
                        )
                    _send_message(channel, {"type": "stopped"})
                    return 0
            except Exception as exc:
                _send_message(
                    channel,
                    {"type": "error", "message": f"{type(exc).__name__}: {exc}"},
                )
    except Exception as exc:
        try:
            _send_message(
                channel,
                {"type": "error", "message": f"{type(exc).__name__}: {exc}"},
            )
        except OSError:
            pass
        return 1
    finally:
        if client is not None and is_moving:
            _stop_client(client)
        if reader is not None:
            reader.close()
        channel.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
