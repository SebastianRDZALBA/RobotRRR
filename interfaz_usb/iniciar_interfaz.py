# Servidor local de la interfaz V6. No requiere internet ni librerias externas.
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import os
import sys
import webbrowser

PORT = 5517

class NoCacheHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def main():
    os.chdir(Path(__file__).resolve().parent)
    try:
        server = ThreadingHTTPServer(("127.0.0.1", PORT), NoCacheHandler)
    except OSError as exc:
        print(f"ERROR: No se pudo abrir el puerto {PORT}: {exc}")
        print("Cierra la otra ventana de la interfaz o libera ese puerto y vuelve a iniciar.")
        print("No se abrira el navegador para evitar mostrar archivos de otro proyecto.")
        return 1
    url = f"http://127.0.0.1:{PORT}/"
    print("Interfaz del brazo V6 - modelo XY negativo (-56 mm), revision verificada")
    print(f"Carpeta servida: {Path.cwd()}")
    print(f"Abre la URL: {url}")
    print("Cierra la consola presionando Ctrl+C. Sin internet.", flush=True)
    webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
