"""Loopback-only collection dashboard. HTTP handlers never control hardware."""
import hmac
import json
import secrets
import threading
import time
import webbrowser
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, unquote

from data_pipeline.data_rerun.arm_pose import ArmModel
from data_pipeline.data_rerun.leader_preview import FOLLOWER_BASE_ROTATION


class Dashboard:
    def __init__(self, expected_hash):
        model = ArmModel.load('follower')
        if model.sha256 != expected_hash:
            raise ValueError('Follower URDF does not match the mapping')
        self.lock = threading.Lock()
        self.state = {'phase': 'waiting', 'allowed': [], 'episodes': [], 'epoch': 0}
        self.command = None
        self.visible = True
        self.token = secrets.token_urlsafe(32)
        self.assets = {}
        root = ET.fromstring(model.urdf_bytes)
        def origin(element):
            node = element.find('origin')
            return {'xyz': [float(v) for v in node.get('xyz', '0 0 0').split()] if node is not None else [0]*3,
                    'rpy': [float(v) for v in node.get('rpy', '0 0 0').split()] if node is not None else [0]*3}
        links = []
        for link in root.findall('link'):
            visuals = []
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None: continue
                uri = urlparse(mesh.get('filename'))
                if uri.scheme != 'file': raise ValueError('Expected resolved local mesh path')
                key = str(len(self.assets))
                self.assets[key] = Path(unquote(uri.path))
                visuals.append({**origin(visual), 'mesh': key, 'scale': [float(v) for v in mesh.get('scale', '1 1 1').split()]})
            links.append({'name': link.get('name'), 'visuals': visuals})
        joints = []
        for joint in root.findall('joint'):
            axis = joint.find('axis')
            joints.append({**origin(joint), 'name': joint.get('name'), 'type': joint.get('type'),
                           'parent': joint.find('parent').get('link'), 'child': joint.find('child').get('link'),
                           'axis': [float(v) for v in axis.get('xyz').split()] if axis is not None else [0,0,1]})
        self.model = {'links': links, 'joints': joints, 'rotation': FOLLOWER_BASE_ROTATION.tolist()}
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def reply(self, code, data, mime='application/json'):
                if not isinstance(data, bytes): data = json.dumps(data, allow_nan=False).encode()
                self.send_response(code)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('X-Frame-Options', 'DENY')
                self.send_header('Content-Security-Policy', "default-src 'self'; frame-ancestors 'none'; style-src 'self' 'unsafe-inline'")
                self.end_headers()
                try: self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError): pass
            def valid_host(self):
                return self.headers.get('Host') in (f'127.0.0.1:{owner.server.server_port}', f'localhost:{owner.server.server_port}')
            def do_GET(self):
                if not self.valid_host(): return self.reply(403, {'error':'Invalid host'})
                path = urlparse(self.path).path
                if path == '/api/state':
                    with owner.lock: state = {**owner.state, 'pending': owner.command is not None}
                    return self.reply(200, state)
                if path == '/api/model': return self.reply(200, owner.model)
                if path.startswith('/mesh/') and path[6:] in owner.assets:
                    try: return self.reply(200, owner.assets[path[6:]].read_bytes(), 'application/octet-stream')
                    except OSError: return self.reply(404, {'error':'Mesh unavailable'})
                files = {'/': ('index.html','text/html; charset=utf-8'), '/app.js': ('app.js','text/javascript'), '/style.css': ('style.css','text/css')}
                if path not in files: return self.reply(404, {'error':'Not found'})
                name, mime = files[path]
                data = (Path(__file__).parent / name).read_text().replace('__CONTROL_TOKEN__', owner.token)
                return self.reply(200, data.encode(), mime)
            def do_POST(self):
                origin = self.headers.get('Origin')
                if not self.valid_host() or origin not in (f'http://127.0.0.1:{owner.server.server_port}', f'http://localhost:{owner.server.server_port}') or not hmac.compare_digest(self.headers.get('X-Control-Token',''), owner.token):
                    return self.reply(403, {'error':'Invalid request origin or token'})
                if self.path != '/api/command': return self.reply(404, {'error':'Not found'})
                try:
                    size = int(self.headers.get('Content-Length','0'))
                    if not 0 < size <= 1024: raise ValueError()
                    body = json.loads(self.rfile.read(size))
                    action = body['action']
                    if action not in ('init','record','stop','quit','abort'): raise ValueError()
                except (ValueError, KeyError, TypeError): return self.reply(400, {'error':'Invalid command'})
                error = None
                with owner.lock:
                    urgent = action in ('quit','abort')
                    if action not in owner.state['allowed']:
                        error = 'Action is unavailable in the current state'
                    elif not urgent and (owner.command is not None or body.get('epoch') != owner.state['epoch']):
                        error = 'State changed; refresh before acting'
                    elif owner.command and owner.command[0] == 'abort':
                        error = 'Abort already pending'
                    else:
                        owner.command = (action, owner.state['epoch'], time.monotonic())
                # Never hold the state lock during socket I/O.
                if error: return self.reply(409, {'error': error})
                self.reply(202, {'accepted':action})
        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True, name='capture-dashboard')
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        print(f'采集控制台: {self.url}', flush=True)
        threading.Thread(target=lambda: webbrowser.open(self.url), daemon=True).start()

    def publish(self, data):
        with self.lock:
            epoch = self.state['epoch'] + int((data['phase'], data['allowed']) != (self.state['phase'], self.state['allowed']))
            self.state = {**data, 'epoch': epoch, 'preview_visible': self.visible}

    def read_command(self):
        with self.lock:
            command, self.command = self.command, None
            if command is None: return None
            action, epoch, stamp = command
            if action not in ('quit','abort') and (epoch != self.state['epoch'] or action not in self.state['allowed'] or time.monotonic()-stamp > 2): return None
        if action == 'abort': raise KeyboardInterrupt
        return {'init':'i','record':'r','stop':'s','quit':'q'}[action]

    def check_abort(self):
        with self.lock:
            abort = self.command is not None and self.command[0] == 'abort'
            if abort: self.command = None
        if abort: raise KeyboardInterrupt

    def show(self): self.visible = True
    def hide(self): self.visible = False
    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)


class DashboardKeys:
    def __init__(self, terminal, dashboard): self.terminal, self.dashboard = terminal, dashboard
    def read(self, timeout_s=0.):
        key = self.dashboard.read_command()
        return key if key is not None else self.terminal.read(timeout_s=min(timeout_s, .03))
