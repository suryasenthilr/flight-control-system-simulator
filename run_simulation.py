"""
AeroSim-Control: Local Server & Launcher
Starts a local HTTP server and automatically opens the application in your default web browser.
"""

import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8080
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

def start_server():
    os.chdir(DIRECTORY)
    # Allow port reuse
    socketserver.TCPServer.allow_reuse_address = True
    
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        url = f"http://localhost:{PORT}/index.html"
        print("=" * 65)
        print("✈  AEROSIM-CONTROL: SIMULATION SERVER RUNNING")
        print("=" * 65)
        print(f"Local Server Address: {url}")
        print("Serving files from:  ", DIRECTORY)
        print("Press Ctrl+C in this terminal to stop the server.")
        print("=" * 65)
        
        # Open in browser automatically
        webbrowser.open(url)
        
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server. Goodbye!")
            httpd.server_close()
            sys.exit(0)

if __name__ == '__main__':
    start_server()
