#!/usr/bin/env python3

import argparse
import errno
import functools
import http.server
import socketserver
import sys


class MeshHTTPServer(socketserver.ThreadingTCPServer):
    """Um thread por conexao.

    POR QUE (2026-09-10): o socketserver.TCPServer atende UMA conexao POR VEZ.
    Um cliente que abre o socket e nao fala nada — RViz morto sem o FIN chegar,
    cabo de rede puxado no meio do download, PC suspenso — deixa o serve_forever
    preso naquela conexao e NENHUM outro cliente e' atendido. Sem timeout isso
    dura ate o keepalive do TCP (2 h por padrao no Linux).

    Medido em bancada: com um cliente calado no ar, o GET de outro cliente
    estourou o timeout de 8 s; no instante em que o calado fechou o socket, o
    MESMO GET respondeu em 0,0 s. Era esse o "as vezes as meshes nao carregam,
    reinicia a Pi (ou o PC) e volta" — nao era falta de CPU: as 14 STL (29 MB)
    saem em 0,32 s quando o servidor esta livre.
    """

    allow_reuse_address = True
    daemon_threads = True      # nao segura o encerramento do launch
    request_queue_size = 32


class MeshRequestHandler(http.server.SimpleHTTPRequestHandler):
    # HTTP/1.1 = keep-alive: o RViz busca as 14 STL numa conexao so', em vez de
    # abrir e fechar uma conexao TCP por arquivo.
    protocol_version = 'HTTP/1.1'

    # Fecha conexao ociosa. E' o que impede um cliente morto de segurar um
    # thread para sempre (com o servidor single-thread, segurava o servidor
    # INTEIRO).
    timeout = 15


def main() -> int:
    parser = argparse.ArgumentParser(description='Serve caramelo_description meshes over HTTP.')
    parser.add_argument('--directory', required=True)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument(
        '--timeout',
        type=float,
        default=MeshRequestHandler.timeout,
        help='segundos de ociosidade antes de fechar a conexao (0 = sem timeout)',
    )
    args = parser.parse_args()

    handler = functools.partial(
        MeshRequestHandler,
        directory=args.directory,
    )
    MeshRequestHandler.timeout = args.timeout if args.timeout > 0 else None

    try:
        with MeshHTTPServer((args.host, args.port), handler) as httpd:
            print(
                f'Serving meshes from {args.directory} on '
                f'http://{args.host}:{args.port}/ '
                f'(threaded, idle timeout={MeshRequestHandler.timeout})',
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
