# generate_next_mission_same_logic.py
# - Path generation logic IDENTICAL to the user's DecomposeAndMerge + lawnmower version.
# - CHANGED: obstacle-aware lawnmower (covers all fragments per scanline)
# - CHANGED: to_local() -> holes-preserving transform
# - CHANGED: boolean ops stabilized with .buffer(0)

import sys
import os
import re
import argparse
import collections
import heapq
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Dict, Set, Any

import numpy as np
import pandas as pd
from shapely.geometry import (
    Polygon, MultiPolygon, LineString, MultiLineString, box, Point, MultiPoint
)
from shapely.ops import unary_union
from pyproj import Transformer

# ----------------- 경로/입출력 -----------------
BASE_DIR = Path(__file__).resolve().parent
LOG_DIR = BASE_DIR / "log"

def find_latest_index(pattern: str) -> int:
    mx = -1
    if not LOG_DIR.exists(): return -1
    for fname in os.listdir(LOG_DIR):
        m = re.match(pattern, fname)
        if m: mx = max(mx, int(m.group(1)))
    return mx

def find_latest_polygon_index() -> int:
    return find_latest_index(r'^polygon(\d+)\.txt$')

def load_home(wp_path: Path) -> Tuple[float, float, float]:
    with open(wp_path, "r", encoding="utf-8") as f:
        f.readline()
        parts = f.readline().strip().split('\t')
        lat, lon, alt = map(float, parts[8:11])
    return lat, lon, alt

def load_area_and_obstacles(txt_path: Path) -> Dict[str, Any]:
    if not txt_path.exists():
        print(f"[오류] 폴리곤 파일 '{txt_path.name}'을 찾을 수 없습니다.", file=sys.stderr)
        sys.exit(1)
    content = txt_path.read_text(encoding="utf-8").lstrip("\ufeff")
    ctx = {"__builtins__": {}, "Polygon": Polygon}
    try:
        exec(content, ctx, ctx)   # polygonN.txt를 그대로 실행 (A, H를 읽음)
    except Exception as e:
        print(f"[오류] 폴리곤 파일 실행 중 오류: {e}", file=sys.stderr)
        sys.exit(1)
    A_poly = ctx.get("A", None)
    H_list = ctx.get("H", [])
    if not isinstance(A_poly, Polygon) or A_poly.is_empty:
        print("[오류] A가 유효한 Polygon이 아닙니다.", file=sys.stderr)
        sys.exit(1)
    polys_H: List[Polygon] = []
    if isinstance(H_list, (list, tuple)):
        for i, p in enumerate(H_list, 1):
            if isinstance(p, Polygon) and (not p.is_empty):
                polys_H.append(p)
            else:
                print(f"[경고] H[{i}] 항목이 유효한 Polygon이 아님 → 건너뜀", file=sys.stderr)
    return {"A": A_poly, "H": polys_H}

def save_polygon_file(path: Path, poly_A: Any, polys_H: List[Polygon]):
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Auto-generated polygon file\n\n")
        def format_poly(p):
            if not isinstance(p, Polygon) or p.is_empty: return ""
            coords_str = ',\n        '.join([f"({lon:.6f}, {lat:.6f})" for lon, lat in p.exterior.coords])
            return f"Polygon([\n        {coords_str}\n    ])"
        if poly_A and not poly_A.is_empty:
            if isinstance(poly_A, Polygon):
                f.write(f"A = {format_poly(poly_A)}\n\n")
            elif isinstance(poly_A, MultiPolygon):
                largest_poly = max(poly_A.geoms, key=lambda p: p.area)
                f.write(f"A = {format_poly(largest_poly)}\n\n")
        if polys_H:
            h_strs = ',\n    '.join([format_poly(p) for p in polys_H])
            f.write(f"H = [\n    {h_strs}\n]")

def save_wp(out_path: Path, home_lat: float, home_lon: float, rel_alt: float, coords_latlon: list):
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("QGC WPL 110\n")
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{home_lat}\t{home_lon}\t{rel_alt}\t1\n")
        for i, (lat, lon) in enumerate(coords_latlon, start=1):
            f.write(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat}\t{lon}\t{rel_alt}\t1\n")

# ----------------- 기하/분할/그래프 (사용자 버전과 동일 논리) -----------------
def Convex(poly: Polygon, angle_tol: float = 1e-6) -> bool:
    if poly.is_empty or (not poly.is_valid) or len(poly.interiors) > 0: return False
    return poly.convex_hull.buffer(1e-9).covers(poly)

def PartitionHorizontal(A: Polygon, H: List[Polygon]) -> List[Polygon]:
    # CHANGED: 안정화
    free = (A.difference(unary_union(H)) if H else A).buffer(0)
    if free.is_empty: return []
    all_coords = []
    for geom in getattr(free, 'geoms', [free]):
        if isinstance(geom, Polygon):
            all_coords.extend(geom.exterior.coords)
            for interior in geom.interiors:
                all_coords.extend(interior.coords)
    if not all_coords: return []
    ys = sorted(list(set(c[1] for c in all_coords)))
    minx, _, maxx, _ = free.bounds
    parts: List[Polygon] = []
    for i in range(len(ys) - 1):
        y0, y1 = ys[i], ys[i + 1]
        if y1 <= y0 + 1e-9: continue
        slab = box(minx - 1.0, y0, maxx + 1.0, y1)
        inter = free.intersection(slab).buffer(0)  # 안정화
        if not inter.is_empty:
            for g in getattr(inter, "geoms", [inter]):
                if isinstance(g, Polygon) and g.area > 1e-6:
                    parts.append(g)
    return parts

def BuildAdjacency(parts: List[Polygon], min_shared: float) -> Dict[int, Set[int]]:
    N = len(parts); adj: Dict[int, Set[int]] = {i: set() for i in range(N)}
    for i in range(N):
        for j in range(i + 1, N):
            inter = parts[i].boundary.intersection(parts[j].boundary)
            if not inter.is_empty and inter.length > min_shared:
                adj[i].add(j); adj[j].add(i)
    return adj

def DFS_Order_With_Levels(adj: Dict[int, Set[int]], root: int, N: int) -> Tuple[List[int], List[int]]:
    if N == 0: return [], []
    levels = [-1] * N
    if root < N and root in adj:
        levels[root] = 0
        queue = collections.deque([root])
        seen = {root}
        while queue:
            u = queue.popleft()
            for v in sorted(list(adj.get(u, set()))):
                if v not in seen:
                    seen.add(v); levels[v] = levels[u] + 1; queue.append(v)
    order: List[int] = []; seen2 = [False] * N
    if root < N and root in adj:
        stack = [root]; seen2[root] = True
        while stack:
            u = stack.pop(); order.append(u)
            for v in sorted(list(adj.get(u, set())), reverse=True):
                if not seen2[v] and levels[v] > levels[u]:
                    seen2[v] = True; stack.append(v)
    return order, levels

@dataclass
class Segment:
    valid: bool
    node_range: Tuple[int, int]
    nodes: List[int]
    edges: List[Tuple[int, int]]

def GenerateSegmentsFromPath(order: List[int], adj: Dict[int, Set[int]], levels: List[int]) -> List[Segment]:
    if not order: return []
    groups: List[List[int]] = []
    cur = [order[0]]
    for i in range(len(order) - 1):
        u, v = order[i], order[i+1]
        cont = (v in adj.get(u, set())) and (levels[v] > levels[u])
        if cont: cur.append(v)
        else: groups.append(cur); cur = [v]
    groups.append(cur)
    segs: List[Segment] = []
    for nodes in groups:
        if not nodes: continue
        edges = []
        for i in range(len(nodes) - 1):
            u, v = nodes[i], nodes[i+1]
            if v in adj.get(u, set()): edges.append((u, v))
        segs.append(Segment(True, (nodes[0], nodes[-1]), nodes, edges))
    return segs

def MergeSegment(parts: List[Polygon], seg_nodes: List[int], seg_edges: List[Tuple[int, int]], angle_tol: float) -> List[Polygon]:
    if not seg_nodes: return []
    poly_map: Dict[int, Polygon] = {nid: parts[nid] for nid in seg_nodes}
    local_adj: Dict[int, Set[int]] = {nid: set() for nid in seg_nodes}
    for (u, v) in seg_edges:
        if u in local_adj and v in local_adj:
            local_adj[u].add(v); local_adj[v].add(u)
    changed = True
    while changed and len(poly_map) > 1:
        changed = False
        current_nodes = [n for n in seg_nodes if n in poly_map]
        for i in range(len(current_nodes) - 1):
            u, v = current_nodes[i], current_nodes[i+1]
            if v not in local_adj.get(u, set()): continue
            U = unary_union([poly_map[u], poly_map[v]]).buffer(0)
            if isinstance(U, Polygon) and Convex(U, angle_tol):
                poly_map[u] = U; del poly_map[v]
                v_neis = list(local_adj.pop(v, set()))
                for w in v_neis:
                    if w in local_adj:
                        local_adj[w].discard(v)
                        if w != u: local_adj[w].add(u); local_adj[u].add(w)
                changed = True
                break
    return list(poly_map.values())

def DecomposeAndMerge(A: Polygon, H: List[Polygon], min_shared: float, angle_tol: float, root: int = 0) -> Dict[str, Any]:
    parts = PartitionHorizontal(A, H)
    adj = BuildAdjacency(parts, min_shared)
    order, levels = DFS_Order_With_Levels(adj, root, len(parts))
    segments = GenerateSegmentsFromPath(order, adj, levels)
    merged_segments: List[List[Polygon]] = []
    for seg in segments:
        Ms = MergeSegment(parts, seg.nodes[:], seg.edges[:], angle_tol) if seg.valid else []
        merged_segments.append(Ms)
    return {"parts": parts, "adj": adj, "order": order, "levels": levels,
            "segments": segments, "merged_segments": merged_segments}

# ----------------- 사용자 버전 lawnmower & 연결 -----------------
def lawnmower(poly_xy: Polygon, spacing_m: float) -> list[tuple[float,float]]:
    """
    CHANGED: obstacle-aware. For each scanline, follow ALL segments (MultiLineString),
    left->right, serpentine within the line to reduce jumps.
    """
    minx, miny, maxx, maxy = poly_xy.bounds
    y = miny
    flip = False
    path_xy: List[tuple[float,float]] = []

    x0, x1 = minx - 1e-9, maxx + 1e-9  # robust intersection

    while y <= maxy + 1e-12:
        cut = LineString([(x0, y), (x1, y)]).intersection(poly_xy)
        if not cut.is_empty:
            if isinstance(cut, LineString):
                segs = [cut]
            else:
                segs = [g for g in getattr(cut, "geoms", []) if isinstance(g, LineString)]
                segs.sort(key=lambda s: min(pt[0] for pt in s.coords))  # left->right
            dir_flag = flip
            for s in segs:
                coords = list(s.coords)
                if dir_flag:
                    coords.reverse()
                path_xy.extend(coords)
                dir_flag = not dir_flag
        y += spacing_m
        flip = not flip
    return path_xy

def build_transition_graph(parts: List[Polygon], adj: Dict[int, Set[int]]):
    node_coords = {}; graph = collections.defaultdict(list); edge_midpoints = {}
    node_id_counter = 0
    for i, neis in adj.items():
        for j in neis:
            if i >= j: continue
            e = (i, j)
            if e not in edge_midpoints:
                shared = parts[i].boundary.intersection(parts[j].boundary)
                if not shared.is_empty:
                    mid = shared.centroid
                    node_coords[node_id_counter] = (mid.x, mid.y)
                    edge_midpoints[e] = node_id_counter
                    node_id_counter += 1
    for pid, _poly in enumerate(parts):
        doors = []
        for nb in adj.get(pid, set()):
            e = (pid, nb) if pid < nb else (nb, pid)
            if e in edge_midpoints: doors.append(edge_midpoints[e])
        for a in range(len(doors)):
            for b in range(a+1, len(doors)):
                u, v = doors[a], doors[b]
                p1 = Point(node_coords[u]); p2 = Point(node_coords[v])
                d = p1.distance(p2)
                graph[u].append((v, d)); graph[v].append((u, d))
    return graph, node_coords, edge_midpoints

def dijkstra_path(graph, node_coords, start_node, end_node):
    pq = [(0, start_node, [])]; seen = set()
    while pq:
        dist, u, path = heapq.heappop(pq)
        if u in seen: continue
        seen.add(u); path = path + [u]
        if u == end_node:
            return [node_coords[n] for n in path]
        for v, w in graph.get(u, []):
            if v not in seen:
                heapq.heappush(pq, (dist + w, v, path))
    return []

def connect_paths_robust(paths: List[LineString], parts: List[Polygon], adj: Dict[int, Set[int]], nav_graph_data) -> LineString:
    if not paths: return LineString()
    if len(paths) == 1: return paths[0]
    graph, node_coords, edge_midpoints = nav_graph_data
    full_path_coords = list(paths[0].coords)
    for i in range(1, len(paths)):
        last_point = Point(full_path_coords[-1])
        nxt = paths[i]
        start_next = Point(nxt.coords[0])
        start_pid = next((j for j, p in enumerate(parts) if p.buffer(1e-9).contains(last_point)), -1)
        end_pid   = next((j for j, p in enumerate(parts) if p.buffer(1e-9).contains(start_next)), -1)
        if start_pid != -1 and end_pid != -1 and start_pid != end_pid:
            temp_g = {k: v[:] for k, v in graph.items()}
            temp_nc = node_coords.copy()
            start_node = len(temp_nc); end_node = start_node + 1
            temp_nc[start_node] = (last_point.x, last_point.y)
            temp_nc[end_node]   = (start_next.x,  start_next.y)
            for nb in adj.get(start_pid, set()):
                e = (start_pid, nb) if start_pid < nb else (nb, start_pid)
                if e in edge_midpoints:
                    door = edge_midpoints[e]
                    d = last_point.distance(Point(temp_nc[door]))
                    temp_g.setdefault(start_node, []).append((door, d))
                    temp_g.setdefault(door, []).append((start_node, d))
            for nb in adj.get(end_pid, set()):
                e = (end_pid, nb) if end_pid < nb else (nb, end_pid)
                if e in edge_midpoints:
                    door = edge_midpoints[e]
                    d = start_next.distance(Point(temp_nc[door]))
                    temp_g.setdefault(end_node, []).append((door, d))
                    temp_g.setdefault(door, []).append((end_node, d))
            trans = dijkstra_path(temp_g, temp_nc, start_node, end_node)
            if trans:
                # 사용자 버전과 동일: 양끝 중복 제거 후 끼워넣기
                full_path_coords.extend(trans[1:-1])
        full_path_coords.extend(list(nxt.coords))
    return LineString(full_path_coords)

def reorder_paths_nearest_neighbor(paths: List[LineString]) -> List[LineString]:
    if len(paths) <= 1: return paths
    unvisited = list(paths); ordered = []
    cur = unvisited.pop(0); ordered.append(cur)
    while unvisited:
        last = Point(cur.coords[-1]); best_d = float('inf'); best_i = -1; rev = False
        for i, p in enumerate(unvisited):
            d0 = last.distance(Point(p.coords[0])); d1 = last.distance(Point(p.coords[-1]))
            if d0 < best_d: best_d = d0; best_i = i; rev = False
            if d1 < best_d: best_d = d1; best_i = i; rev = True
        nxt = unvisited.pop(best_i)
        if rev: nxt = LineString(list(nxt.coords)[::-1])
        ordered.append(nxt); cur = nxt
    return ordered

# ----------------- 메인 -----------------
def main():
    ap = argparse.ArgumentParser(description="Generate UAV mission (same pipeline as user's DecomposeAndMerge).")
    ap.add_argument("--p-human", type=float, default=0.3)
    ap.add_argument("--p-ship",  type=float, default=0.4)
    ap.add_argument("--spacing", type=float, default=60.0)   # 미터 (TMerc)
    ap.add_argument("--rel-alt", type=float, default=10.0)
    args = ap.parse_args()

    LOG_DIR.mkdir(exist_ok=True)

    # 1) 파일 로드
    home_file = LOG_DIR / "home.wp"
    if not home_file.exists():
        print(f"[오류] log/home.wp 파일이 없습니다.", file=sys.stderr); sys.exit(1)

    idx_poly = find_latest_polygon_index()
    print(f"[정보] 최신 polygon 인덱스: {idx_poly}")
    if idx_poly < 0:
        print(f"[오류] log/polygon*.txt 파일이 없습니다. (초기 polygon0.txt 필요)", file=sys.stderr); sys.exit(1)

    poly_file = LOG_DIR / f"polygon{idx_poly}.txt"
    prob_file = LOG_DIR / f"prob{idx_poly}.csv"

    home_lat, home_lon, _ = load_home(home_file)
    poly_data = load_area_and_obstacles(poly_file)
    A_geo, H_geo = poly_data["A"], poly_data["H"]

    print(f"[정보] 기준 파일 로드: home.wp, polygon{idx_poly}.txt")

    # 2) 확률 기반 영역 갱신(있으면), 없으면 기존 영역 전체
    free_space = (A_geo.difference(unary_union(H_geo)) if H_geo else A_geo).buffer(0)  # CHANGED: 안정화
    A_next_geo = free_space
    if prob_file.exists():
        try:
            df = pd.read_csv(prob_file)
            if {"lat","lon","p_human","p_ship"}.issubset(df.columns):
                pts = df[(df.p_human >= args.p_human) & (df.p_ship >= args.p_ship)][["lat","lon"]].to_numpy()
                if len(pts) >= 3:
                    hull = MultiPoint([(lon, lat) for lat, lon in pts]).convex_hull
                    if not hull.is_empty:
                        A_next_geo = free_space.intersection(hull).buffer(0)  # CHANGED: 안정화
                        print("[정보] 확률 폴리곤과 교집합으로 영역 갱신")
        except Exception as e:
            print("[경고] prob 파일 처리 실패, 기존 영역 사용:", e)

    if A_next_geo.is_empty:
        print("[정보] 최종 수색 영역이 비어있어 경로를 생성하지 않습니다."); sys.exit(0)

    # 3) 좌표계 변환 (홈 기준 TMerc, spacing은 미터)
    proj = Transformer.from_crs(
        "EPSG:4326",
        f"+proj=tmerc +lat_0={home_lat} +lon_0={home_lon} "
        "+k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs",
        always_xy=True,
    )
    inv_proj = Transformer.from_crs(proj.target_crs, "EPSG:4326", always_xy=True)

    # CHANGED: 구멍(interiors)까지 보존하여 로컬 좌표로 변환
    def transform_polygon_with_holes(poly: Polygon) -> Polygon | None:
        if not isinstance(poly, Polygon) or poly.is_empty:
            return None
        ext = [proj.transform(lon, lat) for lon, lat in poly.exterior.coords]
        holes = []
        for ring in poly.interiors:
            holes.append([proj.transform(lon, lat) for lon, lat in ring.coords])
        out = Polygon(ext, holes).buffer(0)  # 안정화
        if isinstance(out, Polygon) and not out.is_empty:
            return out
        return None

    def to_local_list(geom) -> List[Polygon]:
        if not geom or geom.is_empty:
            return []
        if isinstance(geom, Polygon):
            p = transform_polygon_with_holes(geom)
            return [p] if p else []
        if hasattr(geom, "geoms"):
            res = []
            for g in geom.geoms:
                if isinstance(g, Polygon):
                    p = transform_polygon_with_holes(g)
                    if p: res.append(p)
            return res
        return []

    # MultiPolygon 가능성 및 홀 보존 고려
    A_xy_list = [g for g in to_local_list(A_next_geo) if g]

    # 4) 영역별: 분해→세그먼트 병합→세그먼트 커버리지→세그먼트 슈퍼패스
    #    그리고 전역 parts/adj를 구축해 최종에도 동일한 connect_paths_robust 로직 사용
    all_segment_super_paths: List[LineString] = []

    global_parts: List[Polygon] = []
    global_adj: Dict[int, Set[int]] = {}
    global_offset = 0

    for area_xy in A_xy_list:
        print(f"\n🚀 영역 처리 중 (면적: {area_xy.area:.1f} m^2)...")

        # *** 사용자 버전과 동일하게 min_shared=0.0 ***
        result = DecomposeAndMerge(area_xy, [], min_shared=0.0, angle_tol=1e-6, root=0)

        # 전역 parts/adj에 merge
        n_local = len(result["parts"])
        global_parts.extend(result["parts"])
        for i, nbrs in result["adj"].items():
            global_adj[global_offset + i] = set(global_offset + j for j in nbrs)
        global_offset += n_local

        # 세그먼트별 병합 폴리곤 -> lawnmower -> 내부 연결
        nav_graph_local = build_transition_graph(result['parts'], result['adj'])
        for seg_polys in result["merged_segments"]:
            if not seg_polys: continue
            indiv_paths = []
            for poly in seg_polys:
                coords = lawnmower(poly, spacing_m=args.spacing)  # 미터 단위 spacing
                if len(coords) > 1:
                    indiv_paths.append(LineString(coords))
            if not indiv_paths: continue
            ordered = reorder_paths_nearest_neighbor(indiv_paths)
            super_path = connect_paths_robust(ordered, result['parts'], result['adj'], nav_graph_local)
            if super_path.length > 0:
                all_segment_super_paths.append(super_path)

    if not all_segment_super_paths:
        print("[오류] 최종 비행 경로 생성에 실패했습니다."); sys.exit(1)

    # 5) 모든 ‘세그먼트 슈퍼패스’를 동일 로직으로 한 번 더 robust 연결
    print("\n🔗 Home에서 시작하여 전체 경로 연결(전역 그래프 사용)...")
    nav_graph_global = build_transition_graph(global_parts, global_adj)

    # Home을 앵커로 넣고 순서 정렬
    home_xy = Point(proj.transform(home_lon, home_lat))
    home_stub = LineString([home_xy, home_xy])
    ordered_all = reorder_paths_nearest_neighbor([home_stub] + all_segment_super_paths)

    # 최종도 connect_paths_robust로!
    final_xy = connect_paths_robust(ordered_all, global_parts, global_adj, nav_graph_global)
    last_pt = Point(final_xy.coords[-1])
    if last_pt.distance(home_xy) > 1e-6:
        final_xy = LineString(list(final_xy.coords) + [(home_xy.x, home_xy.y)])
    # ---------------------------------------

    # 6) 위경도 변환 & 저장
    final_path_latlon = []
    for x, y in final_xy.coords:
        lon, lat = inv_proj.transform(x, y)
        final_path_latlon.append((lat, lon))

    # --- 추가: 첫 점이 home과 사실상 같으면(항상 그럴 가능성 큼) 중복 제거 ---
    if final_path_latlon and abs(final_path_latlon[0][0] - home_lat) < 1e-7 and abs(final_path_latlon[0][1] - home_lon) < 1e-7:
        final_path_latlon = final_path_latlon[1:]

    out_wp = LOG_DIR / f"route{idx_poly + 1}.wp"
    save_wp(out_wp, home_lat, home_lon, args.rel_alt, final_path_latlon)

    out_poly = LOG_DIR / f"polygon{idx_poly + 1}.txt"
    save_polygon_file(out_poly, A_next_geo, H_geo)

    print(f"\n[성공] 저장 완료:")
    print(f" - {out_wp.name} ({len(final_path_latlon)}개의 웨이포인트)")
    print(f" - {out_poly.name}")

if __name__ == "__main__":
    main()