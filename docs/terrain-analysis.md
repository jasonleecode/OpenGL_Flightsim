# 地形系统分析

本程序的地面由**真实地理数据 + GPU 几何裁剪贴图(Geometry Clipmap)**实现,场景是奥地利福拉尔贝格(Vorarlberg)/博登湖(Bodensee)地区的真实地貌。

## 1. 数据层:真实世界的地形瓦片(离线预处理)

`tools/terrain/` 里的脚本从网上下载标准 Web 墨卡托 XYZ 瓦片并拼成 1024x1024 的大图:

- **高度数据**:`create_map.sh` 从 Nextzen/Tilezen 下载 "terrarium" 格式的地形瓦片,高程编码在 RGB 里(`r*256 + g + b/256 - 32768` 米),`decode_heightmap.py` 解码成单通道高度图
- **地表纹理**:从 ArcGIS Online 的 World Imagery 下载卫星照片
- **法线图**:同样从 Tilezen 的 normal 瓦片生成

产物是 `assets/textures/terrain/data/10/536/356/` 下的 4 张图:

| 文件 | 内容 |
| --- | --- |
| `heightmap.png` | 高度图,r 通道 × 3000 米 |
| `normalmap.png` | 法线图 |
| `texture.png` | 卫星照片 |
| `heightmap_encoded.png` | 原始 terrarium 编码高度图 |

目录名 `10/536/356` 就是 zoom 10 的瓦片坐标。整个区域覆盖约 **101.4 km × 101.4 km**(`src/terrain.h:6` 的 `MAX_TILE_SIZE = 50708*4` 除以 `ZOOM_FACTOR = 2`)。画面中的湖就是博登湖,山河都是真实地形。

## 2. 网格层:Geometry Clipmap LOD(`src/terrain.h`)

这是经典的地形渲染技术(GPU Gems 2 的方法):

- 地形被分成 16 个细节层级(`Clipmap` 类,`src/terrain.h:123`),每层级网格间距是上一层的 2 倍(`2^level × 2` 米)
- 每层在相机周围铺一个 5×5 的瓦片环,位置按网格对齐(`calc_base`),相机移动时瓦片整体"吸附"移动而不是重建
- 内层(近处)网格密,外层(远处)网格稀疏;层与层之间用 `Seam`(缝补三角形)和 fixup 条带消除裂缝
- 相机飞得很高时直接跳过细节层(`src/terrain.h:194`),节省性能
- CPU 只摆放平面网格,**真正的起伏在顶点着色器里完成**

## 3. 着色器层(`shaders/terrain.vert` / `terrain.frag`)

- **顶点着色器**:每个顶点根据自己的世界 XZ 坐标算出 UV,从高度图采样,`FragPos.y = 3000 × heightmap.r`(`terrain.vert:70`)——山体就是这样"顶"出来的;法线从法线图采样
- **片元着色器**:卫星照片颜色 × 方向光漫反射(用法线),再混合距离雾(1km 开始,100km 全雾)融入天空背景色(`terrain.frag:39-48`)

## 注意事项

- **湖不是水**,只是卫星照片里的蓝色像素 + 那里海拔低,没有水面模拟
- **没有地形碰撞**——CPU 端没有网格碰撞体,飞机会穿山。唯一的例外是拉升警告,它在 CPU 端单独采样了高度图(`src/main.cpp` 里的 `terrain_height`)
- 想换地区的话,改 `src/terrain.h:19` 的瓦片坐标 `PATH` 并用 `tools/terrain` 的脚本重新下载数据即可(脚本里还注释着卢卡拉、珠峰的坐标)
