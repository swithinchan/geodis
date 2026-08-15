#!/usr/bin/env python3
"""Static HTTP server for the geodis map demos.

Usage:
    python3 scripts/serve.py [port]

Then open:  http://localhost:<port>/scripts/map_viewer.html
"""
import http.server
import os
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8765

# Serve from the project root (parent of scripts/)
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

print(f'geodis server on http://localhost:{PORT}', flush=True)
print('  map viewer:  /scripts/map_viewer.html', flush=True)
print('  3D viewer:   /scripts/three.html', flush=True)
print('  route demo:  /scripts/view.html', flush=True)
http.server.ThreadingHTTPServer(('127.0.0.1', PORT), Handler).serve_forever()
