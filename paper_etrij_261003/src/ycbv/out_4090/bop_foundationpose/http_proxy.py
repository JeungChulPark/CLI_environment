#!/usr/bin/env python3
"""Minimal HTTP CONNECT + plain-HTTP forward proxy (for reverse-tunnelling internet to host .3)."""
import asyncio, sys
from urllib.parse import urlsplit

async def pipe(r, w):
    try:
        while True:
            b = await r.read(65536)
            if not b: break
            w.write(b); await w.drain()
    except Exception: pass
    finally:
        try: w.close()
        except Exception: pass

async def handle(cr, cw):
    try:
        line = await cr.readline()
        if not line: cw.close(); return
        parts = line.decode('latin1').split()
        if len(parts) < 3: cw.close(); return
        method, target, ver = parts[0], parts[1], parts[2]
        headers = []
        while True:
            h = await cr.readline()
            if h in (b'\r\n', b'\n', b''): break
            headers.append(h)
        if method.upper() == 'CONNECT':
            host, _, port = target.rpartition(':')
            sr, sw = await asyncio.wait_for(asyncio.open_connection(host, int(port)), 30)
            cw.write(b'HTTP/1.1 200 Connection established\r\n\r\n'); await cw.drain()
        else:
            u = urlsplit(target)
            host, port = u.hostname, u.port or 80
            sr, sw = await asyncio.wait_for(asyncio.open_connection(host, port), 30)
            path = (u.path or '/') + (('?' + u.query) if u.query else '')
            hs = [x for x in headers if not x.lower().startswith(b'proxy-')]
            if not any(x.lower().startswith(b'connection:') for x in hs):
                hs.append(b'Connection: close\r\n')
            sw.write(f'{method} {path} {ver}\r\n'.encode('latin1') + b''.join(hs) + b'\r\n'); await sw.drain()
        await asyncio.gather(pipe(cr, sw), pipe(sr, cw))
    except Exception as e:
        try:
            cw.write(b'HTTP/1.1 502 Bad Gateway\r\n\r\n'); await cw.drain(); cw.close()
        except Exception: pass

async def main():
    srv = await asyncio.start_server(handle, '127.0.0.1', int(sys.argv[1]) if len(sys.argv) > 1 else 3128)
    async with srv: await srv.serve_forever()
asyncio.run(main())
