#!/usr/bin/python3
import http.server
import socketserver
import os

# create server
PORT = 1340

Handler = http.server.SimpleHTTPRequestHandler
Handler.extensions_map.update({
      ".js": "application/javascript",
})

print("Server open on port", PORT, "\nUse localhost:", PORT)

currPath = os.path.dirname(os.path.abspath(__file__))
os.chdir(currPath)
httpd = socketserver.TCPServer(("", PORT), Handler)
httpd.serve_forever()

