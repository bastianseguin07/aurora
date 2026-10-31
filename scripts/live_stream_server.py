"""
Servidor local para transmitir la vista 3D en vivo a un celular (u otro
dispositivo) en la misma red WiFi, con orbita/zoom controlado desde el
navegador del celular.

No usa un motor 3D externo (three.js, etc): sirve una pagina HTML
autocontenida con un renderer WebGL propio (scripts/web_static/viewer.html),
para no depender de una CDN externa - el sensor Aurora suele conectarse a una
red WiFi propia (SSID SLAMWARE-Aurora-XXXX) que normalmente no tiene salida a
internet.

Dos servidores en threads separados:
- HTTP (http.server, stdlib): sirve la pagina estatica una sola vez.
- WebSocket (libreria 'websockets', loop de asyncio propio): transmite la
  nube base (una vez, o cuando cambia) y la nube en vivo (periodicamente) en
  un formato binario simple.

El resto del pipeline (LiveViewer) llama a broadcast_base()/broadcast_live()
con los mismos puntos/colores que ya calculo para su propia ventana Open3D.
"""

from __future__ import annotations

import asyncio
import http.server
import socket
import struct
import threading
from pathlib import Path

import numpy as np
import qrcode
import websockets
from PIL import Image

STATIC_DIR = Path(__file__).parent / "web_static"
MSG_BASE = 1
MSG_LIVE = 2

# Limites de puntos por frame transmitido. La base se envia una sola vez (o cuando
# cambia), asi que tolera un tope mas alto; el frame en vivo se reenvia varias veces
# por segundo, asi que tiene que ser bastante mas chico para no saturar el WiFi ni el
# renderer del celular. Ambos quedan ademas por debajo de 1 MiB por mensaje, el limite
# de frame que usan por defecto varias librerias de websocket.
MAX_STREAM_POINTS_BASE = 40_000
MAX_STREAM_POINTS_LIVE = 12_000


def _get_lan_ip() -> str:
    """IP local en la red LAN (no se envia trafico real, solo se usa el socket para preguntarle al SO)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("192.168.255.255", 1))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def make_qr_image(url: str, box_size: int = 8) -> Image.Image:
    """Codigo QR (imagen PIL) con la URL de la transmision, para escanear con el celular."""
    qr = qrcode.QRCode(box_size=box_size, border=2)
    qr.add_data(url)
    qr.make(fit=True)
    return qr.make_image(fill_color="black", back_color="white").convert("RGB")


def _downsample(points: np.ndarray, colors: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    if len(points) <= max_points:
        return points, colors
    idx = np.random.choice(len(points), max_points, replace=False)
    return points[idx], colors[idx]


def _encode_frame(msg_type: int, points: np.ndarray, colors: np.ndarray) -> bytes:
    points = np.ascontiguousarray(points, dtype=np.float32)
    colors = np.ascontiguousarray(colors, dtype=np.float32)
    # Header de 8 bytes (dos uint32), no 5 (uint8+uint32): el navegador exige que el
    # offset de un Float32Array sea multiplo de 4, y 5 rompia la lectura en el cliente
    # sin lanzar ningun error visible mas que en la consola del navegador.
    header = struct.pack("<II", msg_type, len(points))
    return header + points.tobytes() + colors.tobytes()


class _StaticHandler(http.server.BaseHTTPRequestHandler):
    ws_port = 0  # se completa por instancia via functools.partial en start()

    def do_GET(self) -> None:  # noqa: N802 (nombre fijado por BaseHTTPRequestHandler)
        html = (STATIC_DIR / "viewer.html").read_text(encoding="utf-8")
        html = html.replace("{{WS_PORT}}", str(self.ws_port))
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args) -> None:  # silencia el log por defecto a stderr
        pass


class LiveStreamServer:
    """Arranca/para el servidor de transmision y reenvia frames a los clientes conectados."""

    def __init__(self, http_port: int = 8080, ws_port: int = 8081, log=print):
        self.http_port = http_port
        self.ws_port = ws_port
        self._log = log

        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws_thread: threading.Thread | None = None
        self._http_server: http.server.ThreadingHTTPServer | None = None
        self._http_thread: threading.Thread | None = None
        self._clients: set = set()
        self._ws_server = None
        self._last_base_frame: bytes | None = None
        self._start_error: Exception | None = None

    def is_running(self) -> bool:
        return bool(self._ws_thread and self._ws_thread.is_alive())

    @property
    def url(self) -> str:
        return f"http://{_get_lan_ip()}:{self.http_port}"

    def start(self) -> str:
        if self.is_running():
            return self.url

        self._start_error = None
        ready = threading.Event()
        self._ws_thread = threading.Thread(target=self._run_ws_loop, args=(ready,), daemon=True)
        self._ws_thread.start()
        if not ready.wait(timeout=5.0):
            self._ws_thread = None
            self._loop = None
            raise RuntimeError("El servidor de WebSocket no respondio a tiempo al iniciar.")
        if self._start_error is not None:
            error = self._start_error
            self._ws_thread = None
            self._loop = None
            raise RuntimeError(f"No se pudo abrir el puerto {self.ws_port} para WebSocket: {error}") from error

        handler = type("_BoundHandler", (_StaticHandler,), {"ws_port": self.ws_port})
        try:
            self._http_server = http.server.ThreadingHTTPServer(("0.0.0.0", self.http_port), handler)
        except OSError as exc:
            self.stop()
            raise RuntimeError(f"No se pudo abrir el puerto {self.http_port} para HTTP: {exc}") from exc
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

        self._log(f"[stream] Servidor iniciado en {self.url} (ws puerto {self.ws_port}).")
        return self.url

    def stop(self) -> None:
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None
        if self._loop is not None and self._ws_thread is not None:
            asyncio.run_coroutine_threadsafe(self._shutdown_ws(), self._loop).result(timeout=5.0)
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._ws_thread.join(timeout=5.0)
        self._loop = None
        self._ws_thread = None
        self._clients.clear()
        self._last_base_frame = None

    def broadcast_base(self, points: np.ndarray, colors: np.ndarray) -> bool:
        """Devuelve True si efectivamente encolo un frame (False si no habia nada que enviar)."""
        if not self.is_running() or len(points) == 0:
            return False
        points, colors = _downsample(points, colors, MAX_STREAM_POINTS_BASE)
        frame = _encode_frame(MSG_BASE, points, colors)
        self._last_base_frame = frame
        self._log(f"[stream] Nube base transmitida: {len(points)} puntos.")
        self._broadcast(frame)
        return True

    def broadcast_live(self, points: np.ndarray, colors: np.ndarray) -> None:
        if not self.is_running() or len(points) == 0:
            return
        points, colors = _downsample(points, colors, MAX_STREAM_POINTS_LIVE)
        self._broadcast(_encode_frame(MSG_LIVE, points, colors))

    # ------------------------------------------------------------------ internals

    def _broadcast(self, frame: bytes) -> None:
        if self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(self._send_to_all(frame), self._loop)

    async def _send_to_all(self, frame: bytes) -> None:
        stale = []
        for client in self._clients:
            try:
                await client.send(frame)
            except websockets.exceptions.ConnectionClosed:
                stale.append(client)
        for client in stale:
            self._clients.discard(client)

    async def _handle_client(self, websocket) -> None:
        self._clients.add(websocket)
        self._log(f"[stream] Celular conectado ({websocket.remote_address[0]}). Clientes activos: {len(self._clients)}.")
        try:
            if self._last_base_frame is not None:
                await websocket.send(self._last_base_frame)
            else:
                self._log("[stream] Aviso: todavia no hay una nube base para enviar al conectar.")
            await websocket.wait_closed()
        finally:
            self._clients.discard(websocket)
            self._log(f"[stream] Celular desconectado. Clientes activos: {len(self._clients)}.")

    async def _shutdown_ws(self) -> None:
        for client in list(self._clients):
            await client.close()
        self._clients.clear()
        if self._ws_server is not None:
            self._ws_server.close()
            await self._ws_server.wait_closed()

    def _run_ws_loop(self, ready: threading.Event) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop

        async def _serve():
            try:
                self._ws_server = await websockets.serve(self._handle_client, "0.0.0.0", self.ws_port)
            except Exception as exc:  # puerto ocupado, permisos, etc.
                self._start_error = exc
            finally:
                ready.set()

        loop.create_task(_serve())
        loop.run_forever()
