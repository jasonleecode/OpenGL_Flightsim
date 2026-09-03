#!/usr/bin/env python3
"""Download a terrain dataset centered on a geographic coordinate.

Fetches 4x4 web mercator tiles (elevation, normals and satellite imagery),
stitches them into the 1024x1024 textures the terrain renderer expects and
installs the dataset into assets/textures/terrain/data/<zoom>/<xtile>/<ytile>/.

Optionally also downloads a high detail satellite texture (texture_hires.png,
used by the map instrument when zooming in) from tiles at zoom + detail.

Usage:
  python3 download_map.py --lat 39.904 --lon 116.407 --name Beijing
  python3 download_map.py --lat 27.988 --lon 86.925 --zoom 10 --name Everest
  python3 download_map.py --lat 39.904 --lon 116.407 --detail 0   # base only

The dataset is discovered automatically the next time the game starts
(map.info metadata file), no code changes needed.
"""

import argparse
import io
import math
import os
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import requests
from PIL import Image

# terrarium elevation and normal tiles from the public aws terrain tile set,
# satellite imagery from the arcgis online world imagery service
TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
NORMAL_URL = "https://s3.amazonaws.com/elevation-tiles-prod/normal/{z}/{x}/{y}.png"
TEXTURE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"

TILES = 4           # dataset is TILES x TILES tiles of 256px -> 1024x1024 pixels
TILE_PX = 256
MAX_ELEVATION = 3000.0  # same scale as the terrain shader (height = 3000 * r)


def lat_lon_to_tile(lat, lon, zoom):
    """tile containing the coordinate, see https://wiki.openstreetmap.org/wiki/Slippy_map"""
    n = 2**zoom
    x = (lon + 180.0) / 360.0 * n
    lat_rad = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return x, y


def fetch_tile(url):
    response = requests.get(url, timeout=30)
    response.raise_for_status()
    return Image.open(io.BytesIO(response.content))


def download_grid(url_fmt, zoom, x_min, y_min, tiles):
    """download a tiles x tiles block of tiles in parallel and stitch it"""
    def job(index):
        tx, ty = index % tiles, index // tiles
        return index, fetch_tile(url_fmt.format(z=zoom, x=x_min + tx, y=y_min + ty))

    out = Image.new("RGBA", (tiles * TILE_PX, tiles * TILE_PX))
    with ThreadPoolExecutor(max_workers=8) as pool:
        for index, tile in pool.map(job, range(tiles * tiles)):
            out.paste(tile.convert("RGBA"), ((index % tiles) * TILE_PX, (index // tiles) * TILE_PX))
    return out


def decode_heightmap(image):
    """decode terrarium rgb encoding into grayscale, 0..255 maps to 0..3000m"""
    data = np.asarray(image, dtype=np.float32)
    elevation = data[..., 0] * 256.0 + data[..., 1] + data[..., 2] / 256.0 - 32768.0
    value = np.clip(elevation / MAX_ELEVATION * 255.0, 0, 255)
    out = np.stack([value, value, value, np.full_like(value, 255)], axis=-1)
    print(f"elevation range: {elevation.min():.0f}m .. {elevation.max():.0f}m")
    return Image.fromarray(out.astype(np.uint8))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--lat", type=float, required=True, help="latitude of the map center")
    parser.add_argument("--lon", type=float, required=True, help="longitude of the map center")
    parser.add_argument("--zoom", type=int, default=10, help="tile zoom level (default 10)")
    parser.add_argument("--detail", type=int, default=2,
                        help="extra zoom levels for the high detail map texture, 0 to skip (default 2)")
    parser.add_argument("--name", type=str, default="Custom", help="display name for the map list")
    parser.add_argument("--out", type=str, default=None, help="output directory (default: assets terrain data dir)")
    args = parser.parse_args()

    zoom = args.zoom
    # tile containing the point, the 4x4 block starts one tile up/left so the
    # point's tile is one of the two center tiles
    cx, cy = lat_lon_to_tile(args.lat, args.lon, zoom)
    x_min, y_min = int(cx) - 1, int(cy) - 1
    print(f"center tile: ({cx:.2f}, {cy:.2f}), dataset tiles: x {x_min}..{x_min + TILES - 1}, "
          f"y {y_min}..{y_min + TILES - 1} at zoom {zoom}")

    print("downloading elevation tiles...")
    heightmap_encoded = download_grid(TERRARIUM_URL, zoom, x_min, y_min, TILES)
    print("downloading normal tiles...")
    normalmap = download_grid(NORMAL_URL, zoom, x_min, y_min, TILES)
    print("downloading satellite imagery...")
    texture = download_grid(TEXTURE_URL, zoom, x_min, y_min, TILES)

    hires = None
    if args.detail > 0:
        detail_tiles = TILES * 2**args.detail
        print(f"downloading high detail imagery ({detail_tiles}x{detail_tiles} tiles at zoom {zoom + args.detail})...")
        hires = download_grid(TEXTURE_URL, zoom + args.detail, x_min * 2**args.detail, y_min * 2**args.detail,
                              detail_tiles)

    out_dir = args.out or os.path.join(os.path.dirname(__file__), "../../assets/textures/terrain/data",
                                       str(zoom), str(x_min), str(y_min))
    os.makedirs(out_dir, exist_ok=True)

    heightmap_encoded.save(os.path.join(out_dir, "heightmap_encoded.png"))
    decode_heightmap(heightmap_encoded).save(os.path.join(out_dir, "heightmap.png"))
    normalmap.save(os.path.join(out_dir, "normalmap.png"))
    texture.convert("RGB").save(os.path.join(out_dir, "texture.png"))
    if hires is not None:
        hires.convert("RGB").save(os.path.join(out_dir, "texture_hires.png"))
    # metadata for the game, which discovers datasets by scanning the data directory
    with open(os.path.join(out_dir, "map.info"), "w") as f:
        f.write(f"name = {args.name}\nzoom = {zoom}\nxtile = {x_min}\nytile = {y_min}\n")
    print(f"dataset written to {out_dir}")
    print(f"restart the game and select '{args.name}' in settings -> map")


if __name__ == "__main__":
    main()
