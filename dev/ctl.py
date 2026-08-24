#!/usr/bin/env python3

import argparse
import json
import socket
import sys

import config


# Control target -> Unix socket
TARGETS = {
    "log": config.LOGGER_CONTROL_SOCKET,

    # Future:
    # "canrx": config.CANRX_CONTROL_SOCKET,
    # "controller": config.CONTROLLER_CONTROL_SOCKET,
}


def send_command(socket_path: str, command: str) -> dict:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

    try:
        sock.settimeout(1.0)
        sock.connect(socket_path)

        sock.sendall((command + "\n").encode("utf-8"))

        response = sock.recv(4096)

        if not response:
            raise RuntimeError("Empty response")

        return json.loads(response.decode("utf-8"))

    finally:
        sock.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mini Celka runtime control utility"
    )

    parser.add_argument(
        "target",
        choices=TARGETS.keys(),
        help="runtime component to control",
    )

    parser.add_argument(
        "command",
        help="command sent to selected component",
    )

    args = parser.parse_args()

    socket_path = TARGETS[args.target]

    try:
        response = send_command(socket_path, args.command)

    except FileNotFoundError:
        print(
            f"ERROR: control socket does not exist: {socket_path}",
            file=sys.stderr,
        )
        return 1

    except ConnectionRefusedError:
        print(
            f"ERROR: nothing is listening on: {socket_path}",
            file=sys.stderr,
        )
        return 1

    except (TimeoutError, socket.timeout):
        print("ERROR: control request timed out", file=sys.stderr)
        return 1

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(response, indent=2))

    return 0


if __name__ == "__main__":
    sys.exit(main())
