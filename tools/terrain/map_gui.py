#!/usr/bin/env python3
"""GUI for download_map.py: pan and zoom a satellite preview, the red rectangle
shows the 4x4 tile block that will be downloaded, then click Download.

Usage:
  python3 map_gui.py
"""

import io
import math
import os
import queue
import shutil
import threading
import time
import tkinter as tk
from concurrent.futures import ThreadPoolExecutor
from tkinter import messagebox, simpledialog
from urllib.parse import urlparse

import download_map as dm
import requests
from PIL import Image, ImageTk

CANVAS = 768
VIEW_ZOOM_MIN, VIEW_ZOOM_MAX = 2, 19
DATASET_ZOOMS = (9, 10, 11, 12)

DATA_DIR = os.path.join(os.path.dirname(__file__), "../../assets/textures/terrain/data")


def scan_maps():
    """all datasets in the terrain data directory that carry a map.info file"""
    maps = []

    def subdirs(path):
        if not os.path.isdir(path):
            return []
        return [os.path.join(path, d) for d in sorted(os.listdir(path)) if os.path.isdir(os.path.join(path, d))]

    for zdir in subdirs(DATA_DIR):
        for xdir in subdirs(zdir):
            for path in subdirs(xdir):
                info_file = os.path.join(path, "map.info")
                if not os.path.isfile(info_file):
                    continue
                info = {}
                with open(info_file) as f:
                    for line in f:
                        if "=" in line:
                            key, val = line.split("=", 1)
                            info[key.strip()] = val.strip()
                maps.append({
                    "name": info.get("name", os.path.basename(path)),
                    "zoom": int(info.get("zoom", os.path.basename(zdir))),
                    "xtile": int(info.get("xtile", os.path.basename(xdir))),
                    "ytile": int(info.get("ytile", os.path.basename(path))),
                    "path": path,
                })
    return maps


class MapGui:
    def __init__(self, root):
        self.root = root
        root.title("Terrain dataset downloader")

        # view state in normalized web mercator coordinates (0..1), zoom independent
        self.center_mx = (116.407 + 180.0) / 360.0  # beijing
        self.center_my = 0.3791
        # selection center (the red rectangle), set by left click
        self.sel_mx, self.sel_my = self.center_mx, self.center_my
        self.view_zoom = 9

        self.tiles = {}          # (zoom, x, y) -> ImageTk.PhotoImage
        self.fetch_pending = set()
        self.fetch_results = queue.Queue()
        self.pool = ThreadPoolExecutor(max_workers=4)
        self.drag_start = None

        # left: map canvas, right: controls
        self.canvas = tk.Canvas(root, width=CANVAS, height=CANVAS, bg="#222")
        self.canvas.pack(side=tk.LEFT)
        panel = tk.Frame(root, padx=10, pady=10)
        panel.pack(side=tk.LEFT, fill=tk.Y)

        # existing datasets: select to preview, delete to remove
        tk.Label(panel, text="Maps").pack(anchor=tk.W)
        self.map_list = tk.Listbox(panel, width=30, height=5, exportselection=False)
        self.map_list.pack(anchor=tk.W)
        self.map_list.bind("<<ListboxSelect>>", self.on_map_select)
        tk.Button(panel, text="Delete selected", command=self.delete_map).pack(anchor=tk.W, pady=(2, 8))

        tk.Label(panel, text="Center coordinate").pack(anchor=tk.W)
        row = tk.Frame(panel)
        row.pack(anchor=tk.W)
        self.lat_var = tk.StringVar(value="39.904")
        self.lon_var = tk.StringVar(value="116.407")
        tk.Entry(row, textvariable=self.lat_var, width=10).pack(side=tk.LEFT)
        tk.Entry(row, textvariable=self.lon_var, width=10).pack(side=tk.LEFT)
        tk.Button(row, text="Go", command=self.go_to).pack(side=tk.LEFT, padx=4)

        tk.Label(panel, text="Dataset zoom").pack(anchor=tk.W, pady=(10, 0))
        self.zoom_var = tk.IntVar(value=10)
        tk.OptionMenu(panel, self.zoom_var, *DATASET_ZOOMS, command=lambda _: self.redraw()).pack(anchor=tk.W)

        self.info_var = tk.StringVar()
        tk.Label(panel, textvariable=self.info_var, justify=tk.LEFT, wraplength=220).pack(anchor=tk.W, pady=10)

        # tile server and zoom levels
        self.server_var = tk.StringVar()
        tk.Label(panel, textvariable=self.server_var, justify=tk.LEFT, wraplength=220, fg="#555").pack(anchor=tk.W)

        # preview tile loading progress
        self.loading_var = tk.StringVar()
        tk.Label(panel, textvariable=self.loading_var, fg="#555").pack(anchor=tk.W, pady=(4, 0))

        self.download_btn = tk.Button(panel, text="Download", command=self.download)
        self.download_btn.pack(anchor=tk.W, pady=(8, 0))
        self.progress_var = tk.StringVar()
        tk.Label(panel, textvariable=self.progress_var, wraplength=220, justify=tk.LEFT).pack(anchor=tk.W)

        self.refresh_maps()

        self.canvas.bind("<ButtonPress-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Button-4>", lambda e: self.zoom_at(e, +1))  # X11 wheel up
        self.canvas.bind("<Button-5>", lambda e: self.zoom_at(e, -1))  # X11 wheel down

        self.canvas.after(100, self.poll_fetches)
        self.redraw()

    # ---- coordinate helpers ------------------------------------------------

    @property
    def scale(self):
        """canvas pixels per normalized mercator unit"""
        return 256 * 2**self.view_zoom

    def to_screen(self, mx, my):
        return ((mx - self.center_mx) * self.scale + CANVAS / 2,
                (my - self.center_my) * self.scale + CANVAS / 2)

    def to_mercator(self, sx, sy):
        return (self.center_mx + (sx - CANVAS / 2) / self.scale,
                self.center_my + (sy - CANVAS / 2) / self.scale)

    @staticmethod
    def mercator_to_lat_lon(mx, my):
        lon = mx * 360.0 - 180.0
        lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * my))))
        return lat, lon

    def dataset_block(self):
        """the 4x4 tile block that a download around the selection point would cover"""
        lat, lon = self.mercator_to_lat_lon(self.sel_mx, self.sel_my)
        zd = self.zoom_var.get()
        cx, cy = dm.lat_lon_to_tile(lat, lon, zd)
        x_min, y_min = int(cx) - 1, int(cy) - 1
        return zd, x_min, y_min

    # ---- map rendering -----------------------------------------------------

    def redraw(self):
        self.canvas.delete("all")
        n = 2**self.view_zoom
        x0 = int(self.center_mx * n - CANVAS / 2 / 256)
        y0 = int(self.center_my * n - CANVAS / 2 / 256)
        x1 = int(self.center_mx * n + CANVAS / 2 / 256) + 1
        y1 = int(self.center_my * n + CANVAS / 2 / 256) + 1

        for ty in range(max(0, y0), min(n, y1 + 1)):
            for tx in range(max(0, x0), min(n, x1 + 1)):
                key = (self.view_zoom, tx, ty)
                sx, sy = self.to_screen(tx / n, ty / n)
                if key in self.tiles:
                    self.canvas.create_image(sx, sy, anchor=tk.NW, image=self.tiles[key])
                elif key not in self.fetch_pending:
                    self.fetch_pending.add(key)
                    self.pool.submit(self.fetch, key)

        # red rectangle: the dataset block
        zd, x_min, y_min = self.dataset_block()
        nd = 2**zd
        ax, ay = self.to_screen(x_min / nd, y_min / nd)
        bx, by = self.to_screen((x_min + dm.TILES) / nd, (y_min + dm.TILES) / nd)
        if ax <= 0 and ay <= 0 and bx >= CANVAS and by >= CANVAS:
            # zoomed into the selection: the block covers the whole canvas
            self.canvas.create_rectangle(2, 2, CANVAS - 2, CANVAS - 2, outline="red", width=3)
        else:
            self.canvas.create_rectangle(ax, ay, bx, by, outline="red", width=2)

        lat, lon = self.mercator_to_lat_lon(self.sel_mx, self.sel_my)
        size_km = 50708.0 * dm.TILES / 2.0**(zd - 9) / 1000.0
        self.info_var.set(f"selection center: {lat:.4f}, {lon:.4f}\n"
                          f"tiles: {zd}/{x_min}/{y_min} .. +{dm.TILES - 1}\n"
                          f"coverage: {size_km:.0f} km x {size_km:.0f} km")

        host = urlparse(dm.TEXTURE_URL).netloc
        self.server_var.set(f"server: {host}\nview zoom: {self.view_zoom}, dataset zoom: {zd}")
        pending = len(self.fetch_pending)
        self.loading_var.set(f"loading {pending} map tiles..." if pending else "map loaded")

    def fetch(self, key):
        zoom, tx, ty = key
        try:
            img = dm.fetch_tile(dm.TEXTURE_URL.format(z=zoom, x=tx, y=ty)).convert("RGB")
            self.fetch_results.put((key, img))
        except Exception:
            self.fetch_results.put((key, None))

    def poll_fetches(self):
        try:
            while True:
                key, img = self.fetch_results.get_nowait()
                self.fetch_pending.discard(key)
                if img is not None:
                    self.tiles[key] = ImageTk.PhotoImage(img)
                if len(self.tiles) > 512:  # bound memory
                    self.tiles.clear()
        except queue.Empty:
            pass
        self.redraw()
        self.canvas.after(100, self.poll_fetches)

    # ---- interaction ---------------------------------------------------------

    def on_press(self, event):
        self.drag_start = (event.x, event.y)
        self.press_pos = (event.x, event.y)

    def on_drag(self, event):
        if self.drag_start is None:
            return
        dx = event.x - self.drag_start[0]
        dy = event.y - self.drag_start[1]
        self.center_mx -= dx / self.scale
        self.center_my -= dy / self.scale
        self.center_mx = min(1.0, max(0.0, self.center_mx))
        self.center_my = min(1.0, max(0.0, self.center_my))
        self.drag_start = (event.x, event.y)
        self.redraw()

    def on_release(self, event):
        # a click (no dragging) moves the selection center to the clicked point
        if self.drag_start is not None and abs(event.x - self.press_pos[0]) < 5 and \
           abs(event.y - self.press_pos[1]) < 5:
            self.sel_mx, self.sel_my = self.to_mercator(event.x, event.y)
            lat, lon = self.mercator_to_lat_lon(self.sel_mx, self.sel_my)
            self.lat_var.set(f"{lat:.4f}")
            self.lon_var.set(f"{lon:.4f}")
        self.drag_start = None
        self.redraw()

    def zoom_at(self, event, direction):
        if direction > 0 and self.view_zoom >= VIEW_ZOOM_MAX:
            return
        if direction < 0 and self.view_zoom <= VIEW_ZOOM_MIN:
            return
        # keep the mercator point under the cursor fixed while zooming
        mx, my = self.to_mercator(event.x, event.y)
        self.view_zoom += direction
        self.center_mx = mx - (event.x - CANVAS / 2) / self.scale
        self.center_my = my - (event.y - CANVAS / 2) / self.scale
        self.redraw()

    def go_to(self):
        try:
            lat, lon = float(self.lat_var.get()), float(self.lon_var.get())
        except ValueError:
            return
        lat_rad = math.radians(lat)
        mx = (lon + 180.0) / 360.0
        my = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0
        self.center_mx, self.center_my = mx, my
        self.sel_mx, self.sel_my = mx, my
        self.redraw()

    # ---- dataset management ------------------------------------------------

    def refresh_maps(self):
        self.maps = scan_maps()
        self.map_list.delete(0, tk.END)
        for m in self.maps:
            self.map_list.insert(tk.END, m["name"])

    def on_map_select(self, _event):
        sel = self.map_list.curselection()
        if not sel:
            return
        m = self.maps[sel[0]]
        # jump the preview to the dataset, selection centered on the block
        self.zoom_var.set(m["zoom"])
        self.view_zoom = max(VIEW_ZOOM_MIN, m["zoom"] - 1)
        n = 2**m["zoom"]
        mx = (m["xtile"] + dm.TILES / 2) / n
        my = (m["ytile"] + dm.TILES / 2) / n
        self.center_mx, self.center_my = mx, my
        self.sel_mx, self.sel_my = mx, my
        lat, lon = self.mercator_to_lat_lon(mx, my)
        self.lat_var.set(f"{lat:.4f}")
        self.lon_var.set(f"{lon:.4f}")
        self.redraw()

    def delete_map(self):
        sel = self.map_list.curselection()
        if not sel:
            return
        m = self.maps[sel[0]]
        if messagebox.askyesno("Delete map", f"Delete '{m['name']}'?\n{m['path']}"):
            shutil.rmtree(m["path"])
            self.refresh_maps()

    def download(self):
        name = simpledialog.askstring("Save map", "Map name:", parent=self.root)
        if not name:
            return
        name = name.strip()
        zd, x_min, y_min = self.dataset_block()
        self.download_btn.config(state=tk.DISABLED)
        threading.Thread(target=self.download_worker, args=(zd, x_min, y_min, name), daemon=True).start()

    def download_worker(self, zoom, x_min, y_min, name):
        total = 3 * dm.TILES * dm.TILES
        detail = 2
        hires_tiles = dm.TILES * 2**detail
        total += hires_tiles * hires_tiles
        done = [0]
        nbytes = [0]
        start = time.time()

        def grid(url_fmt, z, x0, y0, tiles):
            def job(index):
                tx, ty = index % tiles, index // tiles
                response = requests.get(url_fmt.format(z=z, x=x0 + tx, y=y0 + ty), timeout=30)
                response.raise_for_status()
                img = Image.open(io.BytesIO(response.content))
                done[0] += 1
                nbytes[0] += len(response.content)
                elapsed = time.time() - start
                speed = nbytes[0] / elapsed / 1e6 if elapsed > 0 else 0.0
                self.root.after(
                    0, lambda: self.progress_var.set(f"downloading {done[0]}/{total} tiles, {speed:.1f} MB/s"))
                return index, img

            out = Image.new("RGBA", (tiles * dm.TILE_PX, tiles * dm.TILE_PX))
            with ThreadPoolExecutor(max_workers=8) as pool:
                for index, tile in pool.map(job, range(tiles * tiles)):
                    out.paste(tile.convert("RGBA"), ((index % tiles) * dm.TILE_PX, (index // tiles) * dm.TILE_PX))
            return out

        try:
            heightmap_encoded = grid(dm.TERRARIUM_URL, zoom, x_min, y_min, dm.TILES)
            normalmap = grid(dm.NORMAL_URL, zoom, x_min, y_min, dm.TILES)
            texture = grid(dm.TEXTURE_URL, zoom, x_min, y_min, dm.TILES)
            hires = grid(dm.TEXTURE_URL, zoom + detail, x_min * 2**detail, y_min * 2**detail, hires_tiles)

            self.root.after(0, lambda: self.progress_var.set("writing dataset..."))
            out_dir = os.path.join(DATA_DIR, str(zoom), str(x_min), str(y_min))
            os.makedirs(out_dir, exist_ok=True)
            heightmap_encoded.save(os.path.join(out_dir, "heightmap_encoded.png"))
            dm.decode_heightmap(heightmap_encoded).save(os.path.join(out_dir, "heightmap.png"))
            normalmap.save(os.path.join(out_dir, "normalmap.png"))
            texture.convert("RGB").save(os.path.join(out_dir, "texture.png"))
            # note: texture_hires.png is gitignored, it stays local only
            hires.convert("RGB").save(os.path.join(out_dir, "texture_hires.png"))
            with open(os.path.join(out_dir, "map.info"), "w") as f:
                f.write(f"name = {name}\nzoom = {zoom}\nxtile = {x_min}\nytile = {y_min}\n")

            def finish():
                self.progress_var.set(f"done, {nbytes[0] / 1e6:.1f} MB written to {os.path.relpath(out_dir)}\n"
                                      f"restart the game to use '{name}'")
                self.refresh_maps()
                self.download_btn.config(state=tk.NORMAL)
            self.root.after(0, finish)
        except Exception as e:
            import traceback
            traceback.print_exc()

            def fail():
                self.progress_var.set(f"failed: {type(e).__name__}: {e}")
                self.download_btn.config(state=tk.NORMAL)
            self.root.after(0, fail)


def main():
    root = tk.Tk()
    MapGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
