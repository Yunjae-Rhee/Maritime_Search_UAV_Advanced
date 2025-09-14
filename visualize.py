# visualize.py
# - 현재 위치 + 탐색 폴리곤 A + 장애물 H + 최신 경로 + 확률 점 표시
# - 수동 새로고침(버튼)만 사용
# - AUTO-OPEN: 실행 시 기본 브라우저 자동 오픈

import json, re, time, threading, webbrowser
from pathlib import Path
import pandas as pd
from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go

# --- 서버 설정 (원하면 PORT만 바꿔도 됨) ---
HOST = "127.0.0.1"
PORT = 8050
URL  = f"http://{HOST}:{PORT}"

# --- 프로젝트 루트 및 로그 폴더 ---
ROOT = Path(__file__).resolve().parent
LOG  = ROOT / "log"   # 로그 폴더: 코드와 같은 폴더에 log/ 가 있다고 가정

print("[BOOT] ROOT =", ROOT)
print("[BOOT] LOG  =", LOG)

# 최신 인덱스 파일 선택 유틸
def latest(pattern: str):  # ex) r"route(\d+)\.wp"
    mx, pth = -1, None
    if not LOG.exists():
        return None
    for f in LOG.glob("*"):
        m = re.match(pattern, f.name)
        if m:
            try:
                n = int(m.group(1))
                if n > mx:
                    mx, pth = n, f
            except Exception:
                pass
    return pth

# routeN.wp 로드 (QGC WPL)
def load_route():
    f = latest(r"route(\d+)\.wp")
    if not f:
        return [], None
    try:
        lines = f.read_text(encoding="utf-8").splitlines()
        if len(lines) < 2:
            return [], None
        # 홈 (2번째 줄)
        parts = lines[1].strip().split("\t")
        if len(parts) < 11:
            return [], None
        home = (float(parts[8]), float(parts[9]))  # (lat, lon)

        # 웨이포인트 (3번째 줄부터)
        wpts = []
        for line in lines[2:]:
            parts = line.strip().split("\t")
            if len(parts) >= 11:
                wpts.append((float(parts[8]), float(parts[9])))
        return wpts, home
    except Exception as e:
        print("[load_route] error:", e)
        return [], None

# polygonN.txt 에서 A/H 좌표 파싱
def load_polygon_AH_coords():
    f = latest(r"polygon(\d+)\.txt")
    if not f:
        return [], []   # (A_coords, H_polys)
    try:
        txt = f.read_text(encoding="utf-8")

        # A = Polygon([...])
        A_pts = []
        mA = re.search(r'A\s*=\s*Polygon\s*\(\s*\[\s*(.*?)\s*\]\s*\)', txt, flags=re.S)
        if mA:
            body = mA.group(1)
            for m in re.finditer(r'\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)', body):
                lon = float(m.group(1)); lat = float(m.group(2))
                A_pts.append((lat, lon))  # (lat, lon)

        # H = [ Polygon([...]), ... ]
        H_polys = []
        mH = re.search(r'H\s*=\s*\[(.*?)\]\s*$', txt, flags=re.S | re.M)
        if mH:
            H_body = mH.group(1)
            for poly_m in re.finditer(r'Polygon\s*\(\s*\[\s*(.*?)\s*\]\s*\)', H_body, flags=re.S):
                body = poly_m.group(1)
                pts = []
                for m in re.finditer(r'\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)', body):
                    lon = float(m.group(1)); lat = float(m.group(2))
                    pts.append((lat, lon))
                if len(pts) >= 3:
                    H_polys.append(pts)

        return A_pts, H_polys
    except Exception as e:
        print("[load_polygon_AH_coords] error:", e)
        return [], []

# probN.csv 로드
def load_prob(p_h=0.3, p_s=0.4):
    f = latest(r"prob(\d+)\.csv")
    if not f:
        return pd.DataFrame(columns=["lat","lon","p_human","p_ship"])
    try:
        df = pd.read_csv(f)
        if not {"lat","lon","p_human","p_ship"}.issubset(df.columns):
            return pd.DataFrame(columns=["lat","lon","p_human","p_ship"])
        return df  # 필터는 update()에서 적용
    except Exception as e:
        print("[load_prob] error:", e)
        return pd.DataFrame(columns=["lat","lon","p_human","p_ship"])

# 현재 상태 (UAV 위치 등)
def load_state():
    f = LOG / "live_state.json"
    if not f.exists():
        return {}
    try:
        return json.loads(f.read_text(encoding="utf-8"))
    except Exception as e:
        print("[load_state] error:", e)
        return {}

# 비행 궤적
def load_track():
    f = LOG / "track.csv"
    if not f.exists():
        return pd.DataFrame(columns=["timestamp", "lat", "lon", "alt"])
    try:
        df = pd.read_csv(f)
        if not {"lat", "lon"}.issubset(df.columns):
            return pd.DataFrame(columns=["timestamp", "lat", "lon", "alt"])
        return df
    except Exception as e:
        print("[load_track] error:", e)
        return pd.DataFrame(columns=["timestamp", "lat", "lon", "alt"])

# -------------------- Dash 앱 --------------------
app = Dash(__name__)
app.layout = html.Div([
    html.H3("해상 조난사고 수색 UAV", style={
        "fontFamily": "'Noto Sans KR', 'Malgun Gothic', sans-serif",
        "fontWeight": "bold", "fontSize": "28px", "color": "#333"
    }),

    dcc.Graph(id="map", config={"scrollZoom": True}, style={"height": "70vh", "marginBottom": "12px"}),

    html.Div([
        html.Label("사람 발견 확률 ≥", style={"fontFamily": "'Noto Sans', 'Malgun Gothic', sans-serif","fontWeight": "bold"}),
        dcc.Slider(id="p_h", min=0, max=1, step=0.05, value=0.3, tooltip={"always_visible": True}),
        html.Label("선박 발견 확률 ≥", style={"fontFamily": "'Noto Sans', 'Malgun Gothic', sans-serif","fontWeight": "bold"}),
        dcc.Slider(id="p_s", min=0, max=1, step=0.05, value=0.4, tooltip={"always_visible": True}),
        html.Span(id="last_updated", style={"marginLeft":"10px", "color":"#555"})
    ], style={"maxWidth": "600px", "margin": "0 auto"}),

    html.Div([
        html.Button(
            "Apply & Refresh", id="refresh", n_clicks=0,
            style={
                "width": "100%", "padding": "16px 24px", "fontSize": "20px",
                "fontWeight": "600", "borderRadius": "12px",
                "backgroundColor": "#2563eb", "color": "white",
                "border": "none", "cursor": "pointer",
                "boxShadow": "0 4px 12px rgba(0,0,0,0.12)"
            }
        )
    ], style={"maxWidth": "600px", "margin": "12px auto 0"})
])

@app.callback(
    Output("map", "figure"),
    Input("refresh", "n_clicks"),   # 수동 새로고침만
    State("p_h", "value"),
    State("p_s", "value")
)
def update(n_clicks, p_h, p_s):
    # 데이터 로드
    wpts, home = load_route()
    A_coords, H_polys = load_polygon_AH_coords()
    dfp = load_prob(p_h, p_s)
    state = load_state()
    df_track = load_track()

    fig = go.Figure()

    # 비행 궤적
    if not df_track.empty and {"lat", "lon"}.issubset(df_track.columns):
        fig.add_trace(go.Scattermapbox(
            lat=df_track["lat"], lon=df_track["lon"],
            mode="lines", name="Track",
            line={"color": "rgba(0,0,255,1)", "width": 3}
        ))

    # 경로 (markers+lines)
    if wpts:
        fig.add_trace(go.Scattermapbox(
            lat=[p[0] for p in wpts], lon=[p[1] for p in wpts],
            mode="markers+lines", name="Route", marker={"size": 6},
            line={"color": "rgba(0,0,255,0.3)", "width": 3}
        ))

    # 수색 영역 A (면 채움)
    if len(A_coords) >= 3:
        plat, plon = zip(* (A_coords + [A_coords[0]]) )
        fig.add_trace(go.Scattermapbox(
            lat=plat, lon=plon,
            mode="lines", name="Polygon A",
            fill="toself", fillcolor="rgba(255,0,0,0.18)",
            line={"width": 1, "color": "red"}
        ))

    # 장애물 H
    for i, pts in enumerate(H_polys, 1):
        if len(pts) < 3:
            continue
        latp, lonp = zip(* (pts + [pts[0]]) )
        fig.add_trace(go.Scattermapbox(
            lat=latp, lon=lonp,
            mode="lines", name=f"Obstacle {i}",
            fill="toself", fillcolor="rgba(80,80,80,0.35)",
            line={"width": 1, "color": "black"}
        ))

    # 확률 점 (필터)
    human_df = dfp[(dfp["p_human"] >= p_h) & (dfp["p_ship"] < p_s)]
    ship_df  = dfp[(dfp["p_ship"]  >= p_s) & (dfp["p_human"] < p_h)]
    both_df  = dfp[(dfp["p_ship"]  >= p_s) & (dfp["p_human"] >= p_h)]

    if not human_df.empty:
        fig.add_trace(go.Scattermapbox(
            lat=human_df["lat"], lon=human_df["lon"],
            mode="markers", name="Human", marker={"size": 8, "color": "red"}
        ))
    if not ship_df.empty:
        fig.add_trace(go.Scattermapbox(
            lat=ship_df["lat"], lon=ship_df["lon"],
            mode="markers", name="Ship", marker={"size": 8, "color": "blue"}
        ))
    if not both_df.empty:
        fig.add_trace(go.Scattermapbox(
            lat=both_df["lat"], lon=both_df["lon"],
            mode="markers", name="Both", marker={"size": 8, "color": "purple"}
        ))

    # 지도 중심
    if ("lat" in state) and ("lon" in state):
        try:
            lat_now, lon_now = float(state["lat"]), float(state["lon"])
            fig.add_trace(go.Scattermapbox(
                lat=[lat_now], lon=[lon_now],
                mode="markers+text", name="UAV",
                text=["UAV"], textposition="top center",
                marker={"size": 14, "color": "orange"}
            ))
            center = [lat_now, lon_now]
        except:
            center = [wpts[0][0], wpts[0][1]] if wpts else [37.5665, 126.9780]
    elif wpts:
        center = [wpts[0][0], wpts[0][1]]
    else:
        center = [37.5665, 126.9780]

    fig.update_layout(
        mapbox={
            "style": "open-street-map",   # Mapbox 토큰 없이 사용 가능
            "center": {"lat": center[0], "lon": center[1]},
            "zoom": 14
        },
        margin={"l":0,"r":0,"t":0,"b":0},
        legend={"orientation": "h", "yanchor": "bottom", "y": 0.01}
    )
    return fig

if __name__ == "__main__":
    # AUTO-OPEN: 서버가 뜬 뒤 기본 브라우저를 자동으로 연다.
    def _open():
        # 약간의 딜레이를 주어 서버가 준비될 시간을 확보
        time.sleep(0.7)
        try:
            webbrowser.open(URL)
        except Exception:
            pass

    threading.Thread(target=_open, daemon=True).start()
    # 실행 후 브라우저에서 자동으로 열림 (수동으로는 URL 참조)
    app.run(debug=False, host=HOST, port=PORT)