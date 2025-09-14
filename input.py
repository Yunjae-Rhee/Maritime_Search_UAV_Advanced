# uav_dashboard_local_save.py
# - 단일 파일: 내장 HTML을 로컬 서버로 서빙
# - "파일 저장" 클릭 시 브라우저가 /save 로 POST, 파이썬이 ./log 에 자동 저장

import http.server
import socketserver
import webbrowser
import threading
import sys
import time
import json
from pathlib import Path
import re

SAVE_ROOT = Path(__file__).parent.resolve()  # 기준 경로
DEFAULT_SUBDIR = "log"                       # 저장 하위폴더

HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width,initial-scale=1.0"/>
    <title>UAV 경로 계획 대시보드 v6 (서버 자동 저장)</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css"/>
    <link rel="stylesheet" href="https://unpkg.com/leaflet-geosearch@3.11.0/dist/geosearch.css"/>
    <style>
        html, body, #map { height: 100%; width: 100%; margin: 0; padding: 0; overflow: hidden; }
        #control-panel {
            position: absolute; top: 10px; right: 10px; z-index: 1000;
            background: rgba(255,255,255,.95); padding: 15px; border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,.2); width: 380px; max-height: 95vh; overflow-y: auto;
            font-family: -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
        }
        textarea { width: 95%; height: 120px; margin-top: 5px; font-family: monospace; font-size: 12px;
                   border: 1px solid #ddd; border-radius: 4px; padding: 8px; resize: vertical; }
        .action-button { border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; font-size: 16px;
                         width: 100%; margin-top: 10px; transition: background-color .2s; }
        .primary { background-color: #007bff; color: #fff; }
        .primary:hover:enabled { background-color: #0056b3; }
        .secondary { background-color: #28a745; color: #fff; }
        .secondary:hover:enabled { background-color: #218838; }
        .danger { background-color: #dc3545; color: #fff; }
        .danger:hover { background-color: #c82333; }
        .success { background-color: #28a745; color: #fff; }
        .success:hover:enabled { background-color: #218838; }
        .action-button:disabled { background-color: #ccc; cursor: not-allowed; }
        .action-button.active { background-color: #6c757d; opacity: .8; }
        #status { padding: 10px; margin-bottom: 10px; background: #e9ecef; border-radius: 4px; text-align: center;
                  font-size: 14px; font-weight: bold; }
        .leaflet-draw { display: none; }
        small.muted { color: #666; }
        input[type=text]{ width: 100%; padding: 8px; border:1px solid #ddd; border-radius:4px; }
        .row{ display:flex; gap:8px; }
        .row > div{ flex:1; }
    </style>
</head>
<body>
<div id="map"></div>

<div id="control-panel">
    <h2>UAV 경로 계획</h2>
    <div id="status">원하는 작업을 선택하세요.</div>

    <button id="home-btn" class="action-button primary" onclick="enterMode('setHome')">① 홈 위치 선택하기</button>
    <button id="area-btn" class="action-button primary" onclick="enterMode('drawArea')">② 수색 영역 그리기</button>
    <button id="obstacle-btn" class="action-button secondary" onclick="enterMode('drawObstacle')" disabled>③ 장애물 추가하기</button>
    <button class="action-button danger" onclick="clearAll()">전체 초기화</button>

    <hr style="margin: 16px 0">

    <div class="row">
        <div>
            <label><strong>저장 폴더</strong></label>
            <input id="subdir" type="text" value="log" />
            <small class="muted">기본값: ./log</small>
        </div>
        <div>
            <label><strong>상대 고도</strong></label>
            <input id="relalt" type="text" value="10.0" />
            <small class="muted">home.wp ALT</small>
        </div>
    </div>

    <div class="output-group" style="margin-top:10px">
        <label for="polygonOutput"><strong>데이터 미리보기 (저장될 내용)</strong></label>
        <textarea id="polygonOutput" readonly></textarea>
    </div>

    <button id="save-btn" class="action-button success" onclick="saveToServer()" disabled>서버에 저장</button>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
<script src="https://unpkg.com/leaflet-geosearch@3.11.0/dist/geosearch.umd.js"></script>
<script>
let map, homeMarker=null, areaLayer=null, drawControl, statusPanel;
let obstacleLayers = [];
let currentMode = 'idle';
let homeBtn, areaBtn, obstacleBtn, saveBtn;

statusPanel = document.getElementById('status');
homeBtn = document.getElementById('home-btn');
areaBtn = document.getElementById('area-btn');
obstacleBtn = document.getElementById('obstacle-btn');
saveBtn = document.getElementById('save-btn');

map = L.map('map').setView([37.5665, 126.9780], 13);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom:19, attribution:'© OpenStreetMap'}).addTo(map);

const searchControl = new GeoSearch.GeoSearchControl({
    provider: new GeoSearch.OpenStreetMapProvider(),
    style: 'bar', autoClose: true, keepResult: true
});
map.addControl(searchControl);

const drawnItems = new L.FeatureGroup().addTo(map);
drawControl = new L.Control.Draw({
    edit: { featureGroup: drawnItems },
    draw: { polygon: {}, polyline:false, rectangle:false, circle:false, marker:false, circlemarker:false }
});
map.addControl(drawControl);

// 클릭으로 홈 지정
map.on('click', e => {
    if (currentMode === 'setHome') { setHome(e.latlng); enterMode('idle'); }
});

// 도형 생성
map.on(L.Draw.Event.CREATED, e => {
    const layer = e.layer;
    if (currentMode === 'drawArea') {
        if (areaLayer) drawnItems.removeLayer(areaLayer);
        areaLayer = layer;
        areaLayer.setStyle({ color:'#007bff', fillColor:'#007bff' });
    } else if (currentMode === 'drawObstacle') {
        obstacleLayers.push(layer);
        layer.setStyle({ color:'#dc3545', fillColor:'#dc3545' });
    }
    drawnItems.addLayer(layer);
    enterMode('idle');
});

// 편집/삭제
map.on('draw:editstop draw:deletestop', () => {
    if (areaLayer && !drawnItems.hasLayer(areaLayer)) { areaLayer = null; }
    obstacleLayers = obstacleLayers.filter(layer => drawnItems.hasLayer(layer));
    enterMode('idle');
});

function enterMode(mode){
    // 기존 draw 모드 비활성화
    if (drawControl._toolbars.draw._activeMode) {
        drawControl._toolbars.draw._modes.polygon.handler.disable();
    }
    currentMode = mode; updateUIState();
    [homeBtn, areaBtn, obstacleBtn].forEach(btn => btn.classList.remove('active'));
    if (mode==='setHome') homeBtn.classList.add('active');
    if (mode==='drawArea') areaBtn.classList.add('active');
    if (mode==='drawObstacle') obstacleBtn.classList.add('active');

    switch(mode){
        case 'idle':
            statusPanel.textContent = "원하는 작업을 선택하세요.";
            homeBtn.textContent = homeMarker ? "다시 홈 위치 선택하기" : "① 홈 위치 선택하기";
            areaBtn.textContent = areaLayer ? "다시 수색 영역 그리기" : "② 수색 영역 그리기";
            break;
        case 'setHome':
            statusPanel.textContent = "지도 위를 클릭하여 홈 위치를 지정하세요.";
            break;
        case 'drawArea':
            if(areaLayer) { drawnItems.removeLayer(areaLayer); areaLayer = null; }
            new L.Draw.Polygon(map, { shapeOptions:{ color:'#007bff' } }).enable();
            statusPanel.textContent = "지도에 수색 영역을 그리세요.";
            break;
        case 'drawObstacle':
            new L.Draw.Polygon(map, { shapeOptions:{ color:'#dc3545' } }).enable();
            statusPanel.textContent = "지도에 장애물 영역을 그리세요.";
            break;
    }
}

function updateUIState(){
    obstacleBtn.disabled = !areaLayer;
    saveBtn.disabled = !(homeMarker && areaLayer);
    updatePreview();
}

function setHome(latlng){
    if (homeMarker) map.removeLayer(homeMarker);
    homeMarker = L.marker(latlng).addTo(map);
}

// 포맷터
function formatPolygon(latlngs){
    let ring = latlngs[0];
    if (!ring || ring.length < 3) return "Polygon([])";
    const first = ring[0], last = ring[ring.length-1];
    if (first.lat.toFixed(6) === last.lat.toFixed(6) &&
        first.lng.toFixed(6) === last.lng.toFixed(6)) {
        ring = ring.slice(0,-1);
    }
    const coords = ring.map(p => `(${p.lng.toFixed(6)}, ${p.lat.toFixed(6)})`);
    return `Polygon([\n        ${coords.join(',\n        ')}\n    ])`;
}

function updatePreview(){
    let outputText = '';
    if (homeMarker) {
        const lat = homeMarker.getLatLng().lat.toFixed(6), lon = homeMarker.getLatLng().lng.toFixed(6);
        const alt = parseFloat(document.getElementById('relalt').value || '10.0').toFixed(1);
        outputText += `## Home Location (.wp)\nQGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t${lat}\t${lon}\t${alt}\t1\n\n`;
    }
    if (areaLayer) {
        outputText += `## Polygons (.txt)\nA = ${formatPolygon(areaLayer.getLatLngs())}\n\n`;
    }
    if (obstacleLayers.length > 0) {
        const hPolys = obstacleLayers.map(layer => formatPolygon(layer.getLatLngs()));
        outputText += `H = [\n    ${hPolys.join(',\n    ')}\n]`;
    }
    document.getElementById('polygonOutput').value = outputText || '데이터가 없습니다.';
}

async function saveToServer(){
    if (!homeMarker || !areaLayer) {
        alert("홈 위치와 수색 영역을 모두 지정해야 합니다.");
        return;
    }
    const lat = homeMarker.getLatLng().lat.toFixed(6);
    const lon = homeMarker.getLatLng().lng.toFixed(6);
    const alt = parseFloat(document.getElementById('relalt').value || '10.0').toFixed(1);

    const wpContent = `QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\t${lat}\t${lon}\t${alt}\t1\n`;

    let polyContent = `A = ${formatPolygon(areaLayer.getLatLngs())}\n\n`;
    if (obstacleLayers.length > 0) {
        const hPolys = obstacleLayers.map(layer => formatPolygon(layer.getLatLngs()));
        polyContent += `H = [\n    ${hPolys.join(',\n    ')}\n]`;
    }

    const subdir = (document.getElementById('subdir').value || 'log').trim();

    try{
        statusPanel.textContent = "서버에 저장 중...";
        const resp = await fetch('/save', {
            method: 'POST',
            headers: {'Content-Type':'application/json'},
            body: JSON.stringify({
                subdir,
                files: [
                    {name: 'home.wp', content: wpContent},
                    {name: 'polygon0.txt', content: polyContent}
                ]
            })
        });
        if(!resp.ok){
            const t = await resp.text();
            throw new Error(`HTTP ${resp.status}: ${t}`);
        }
        const data = await resp.json();
        statusPanel.textContent = `저장 완료: ${data.saved.join(', ')}`;
    }catch(err){
        console.error(err);
        statusPanel.textContent = `저장 실패: ${err.message}`;
        alert("저장 중 오류가 발생했습니다.\n" + err.message);
    }
}

function clearAll(){
    if (homeMarker) { map.removeLayer(homeMarker); homeMarker = null; }
    const toRemove = [];
    map.eachLayer(function(layer){
        if (!(layer instanceof L.TileLayer)) {
            if (layer instanceof L.Marker || layer instanceof L.Polygon) toRemove.push(layer);
        }
    });
    toRemove.forEach(l => map.removeLayer(l));
    areaLayer = null; obstacleLayers = [];
    enterMode('idle');
    statusPanel.textContent = '초기화 완료. 다시 시작하세요.';
}

enterMode('idle');
</script>
</body>
</html>
"""

SAFE_NAME = re.compile(r"^[\w\-.]+$")

class Handler(http.server.BaseHTTPRequestHandler):
    def _send_json(self, code, obj):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            content = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/health":
            msg = b"ok"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(msg)))
            self.end_headers()
            self.wfile.write(msg)
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        if self.path != "/save":
            self.send_error(404, "Not Found")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length)
            payload = json.loads(raw.decode("utf-8"))

            subdir = payload.get("subdir") or DEFAULT_SUBDIR
            files = payload.get("files") or []

            saved_paths = []
            target_dir = (SAVE_ROOT / subdir).resolve()

            # ./ 안에서만 저장 (경로 탈출 방지)
            if SAVE_ROOT not in target_dir.parents and target_dir != SAVE_ROOT:
                self._send_json(400, {"error": "Invalid subdir"})
                return

            target_dir.mkdir(parents=True, exist_ok=True)

            for f in files:
                name = f.get("name", "")
                content = f.get("content", "")
                if not SAFE_NAME.match(name):
                    self._send_json(400, {"error": f"Invalid filename: {name}"})
                    return
                out_path = target_dir / name
                out_path.write_text(content, encoding="utf-8")
                saved_paths.append(str(out_path.relative_to(SAVE_ROOT)))

            self._send_json(200, {"saved": saved_paths})
        except Exception as e:
            self._send_json(500, {"error": str(e)})

    def log_message(self, fmt, *args):
        sys.stdout.write("[HTTP] " + (fmt % args) + "\n")

def run(port=8000, open_browser=True):
    with socketserver.TCPServer(("127.0.0.1", port), Handler) as httpd:
        url = f"http://127.0.0.1:{port}"
        print(f"\nServing UAV dashboard at {url}")
        if open_browser:
            threading.Thread(target=lambda: (time.sleep(0.4), webbrowser.open(url)), daemon=True).start()
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down...")

if __name__ == "__main__":
    run(port=8000, open_browser=True)