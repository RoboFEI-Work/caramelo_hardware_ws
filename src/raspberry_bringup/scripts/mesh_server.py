#!/usr/bin/env python3

import argparse
import errno
import functools
import http.server
import socketserver
import sys


class ReusableTCPServer(socketserver.TCPServer):
    allow_reuse_address = True


def main() -> int:
    parser = argparse.ArgumentParser(description='Serve caramelo_description meshes over HTTP.')
    parser.add_argument('--directory', required=True)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()

    handler = functools.partial(
        http.server.SimpleHTTPRequestHandler,
        directory=args.directory,
    )

    try:
        with ReusableTCPServer((args.host, args.port), handler) as httpd:
            print(
                f'Serving meshes from {args.directory} on '
                f'http://{args.host}:{args.port}/',
                flush=True,
            )
            httpd.serve_forever()
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(
                f'Mesh server port {args.port} is already in use; '
                'assuming an existing mesh server is active.',
                flush=True,
            )
            return 0
        raise
    except KeyboardInterrupt:
        return 0

    return 0


if __name__ == '__main__':
    sys.exit(main())
