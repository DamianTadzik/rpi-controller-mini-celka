#!/usr/bin/env python3

import socket
import sys

import config


def main() -> int:
    sock = socket.socket(
        socket.AF_UNIX,
        socket.SOCK_DGRAM,
    )

    try:
        sock.sendto(
            b"rotate",
            config.LOGGER_CONTROL_SOCKET,
        )

    except FileNotFoundError:
        print(
            f"ERROR: logger socket does not exist: "
            f"{config.LOGGER_CONTROL_SOCKET}",
            file=sys.stderr,
        )
        return 1

    except Exception as exc:
        print(
            f"ERROR: failed to request log rotation: {exc}",
            file=sys.stderr,
        )
        return 1

    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
    