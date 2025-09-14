# run_mission.py — ADV-only mission runner with DRY-RUN simulator
# - 초기 트리거: home.wp + polygon*.txt "둘 다" 준비 후 경로 생성
# - 이후 트리거: prob*.csv 또는 polygon*.txt 변경 시 경로 재계산
# - input.py: 8000 포트가 비어 있으면 자동 실행, 이미 점유 중이면 실행하지 않고 기다림
# - route_update_advanced.py: --no-upload 제거

import os, sys, time, signal, re, json, threading, subprocess, argparse, math, socket, contextlib
from pathlib import Path
from datetime import datetime, timezone

# MAVLink는 실제 연결 때만 import
try:
    from pymavlink import mavutil
except Exception:
    mavutil = None

BASE_DIR = Path(__file__).resolve().parent
LOG_DIR  = BASE_DIR / "log"
PY       = sys.executable

INPUT_APP        = BASE_DIR / "input.py"                 # home.wp / polygon*.txt 저장 UI
ROUTE_UPDATE_ADV = BASE_DIR / "route_update_advanced.py" # 초기 & 이후 모두 여기로 갱신
IDENTIFIER       = BASE_DIR / "identifier.py"            # (선택) prob 생성기

CONN_STR = "udp:127.0.0.1:14550"
BAUD     = 115200
REL_ALT  = 10.0

POLL_SEC = 2.0
NO_UPDATE_EXIT_CODE = 99
HOME_WAIT_HINT_SEC  = 0.2

# -------------------- 유틸 --------------------
def is_port_free(port: int) -> bool:
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
            return True
        except OSError:
            return False

def file_is_stable(path: Path, settle_ms: int = 300) -> bool:
    if not path.exists() or path.stat().st_size == 0:
        return False
    s1, t1 = path.stat().st_size, path.stat().st_mtime_ns
    time.sleep(settle_ms / 1000.0)
    s2, t2 = path.stat().st_size, path.stat().st_mtime_ns
    return (s1, t1) == (s2, t2)

def latest_index_by_pattern(pattern: str) -> int:
    mx = -1
    if not LOG_DIR.exists():
        return -1
    for p in LOG_DIR.glob("*"):
        m = re.match(pattern, p.name)
        if m:
            try:
                mx = max(mx, int(m.group(1)))
            except:
                pass
    return mx

def latest_prob_index() -> int:
    return latest_index_by_pattern(r"prob(\d+)\.csv$")

def latest_polygon_index() -> int:
    return latest_index_by_pattern(r"polygon(\d+)\.txt$")

def latest_route_file() -> Path | None:
    mx, latest = -1, None
    if not LOG_DIR.exists():
        return None
    for p in LOG_DIR.glob("route*.wp"):
        m = re.match(r"route(\d+)\.wp$", p.name)
        if m:
            n = int(m.group(1))
            if n > mx:
                mx, latest = n, p
    return latest

def parse_wp_latlonalt_list(wp_path: Path):
    pts = []
    with open(wp_path, "r", encoding="utf-8") as f:
        header = f.readline().strip()
        if header != "QGC WPL 110":
            raise ValueError("Unsupported .wp format (need QGC WPL 110)")
        for line in f:
            parts = line.strip().split("\t")
            if len(parts) < 12:
                continue
            lat = float(parts[8]); lon = float(parts[9]); alt = float(parts[10])
            pts.append((lat, lon, alt))
    return pts[1:]  # 0번(홈) 제외

def wait_for_home_wp(timeout_sec: float | None = None) -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    target = LOG_DIR / "home.wp"
    t0 = time.time()
    while True:
        if target.exists() and file_is_stable(target):
            print(f"[run] home.wp 감지: {target}")
            return target
        if timeout_sec is not None and (time.time() - t0) > timeout_sec:
            raise TimeoutError("home.wp 생성 대기 타임아웃")
        time.sleep(HOME_WAIT_HINT_SEC)

def wait_for_polygon_txt(timeout_sec: float | None = None) -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    t0 = time.time()
    while True:
        idx = latest_polygon_index()
        if idx >= 0:
            p = LOG_DIR / f"polygon{idx}.txt"
            if p.exists() and file_is_stable(p):
                print(f"[run] polygon 감지: {p}")
                return p
        if timeout_sec is not None and (time.time() - t0) > timeout_sec:
            raise TimeoutError("polygon*.txt 생성 대기 타임아웃")
        time.sleep(HOME_WAIT_HINT_SEC)

def write_live_and_track(lat: float, lon: float, alt: float | None):
    LOG_DIR.mkdir(exist_ok=True)
    state_path = LOG_DIR / "live_state.json"
    track_path = LOG_DIR / "track.csv"
    if not track_path.exists():
        track_path.write_text("timestamp,lat,lon,alt\n", encoding="utf-8")
    now = datetime.now(timezone.utc).isoformat()
    state = {"timestamp": now, "lat": lat, "lon": lon, "alt": alt}
    state_path.write_text(json.dumps(state), encoding="utf-8")
    with open(track_path, "a", encoding="utf-8") as f:
        f.write(f"{now},{lat},{lon},{'' if alt is None else alt}\n")

# -------------------- DRY-RUN 시뮬레이터 --------------------
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(a))

def move_towards(lat1, lon1, lat2, lon2, step_m):
    dist = haversine_m(lat1, lon1, lat2, lon2)
    if dist <= 1e-6 or step_m >= dist:
        return lat2, lon2, dist
    ratio = step_m / dist
    return lat1 + (lat2-lat1)*ratio, lon1 + (lon2-lon1)*ratio, dist - step_m

class RouteSimulator:
    def __init__(self, speed_mps=12.0, tick=0.5, default_alt=REL_ALT):
        self.speed = float(speed_mps)
        self.tick = float(tick)
        self.alt  = float(default_alt)
        self._lock = threading.Lock()
        self._route = []  # [(lat,lon,alt), ...]
        self._cur_idx = 0
        self._pos = None
        self._stop = threading.Event()
        self._thr  = threading.Thread(target=self._run, daemon=True)

    def set_route(self, latlonalt_list):
        if not latlonalt_list:
            return
        with self._lock:
            self._route = list(latlonalt_list)
            if self._pos is None:
                lat, lon, alt = self._route[0]
                self._pos = (lat, lon, alt if alt is not None else self.alt)
                self._cur_idx = 1 if len(self._route) > 1 else 0
            else:
                self._cur_idx = 1 if len(self._route) > 1 else 0

    def start(self):
        self._thr.start()

    def stop(self):
        self._stop.set()
        self._thr.join(timeout=2.0)

    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                if not self._route:
                    time.sleep(self.tick); continue
                if self._cur_idx >= len(self._route):
                    lat, lon, alt = self._pos
                    write_live_and_track(lat, lon, alt)
                    time.sleep(self.tick); continue
                cur_lat, cur_lon, cur_alt = self._pos
                tgt_lat, tgt_lon, tgt_alt = self._route[self._cur_idx]
            step = self.speed * self.tick
            nlat, nlon, rem = move_towards(cur_lat, cur_lon, tgt_lat, tgt_lon, step)
            if tgt_alt is None:
                tgt_alt = self.alt
            nalt = cur_alt + (tgt_alt - cur_alt) * min(1.0, (step / max(1.0, step + rem)))
            write_live_and_track(nlat, nlon, nalt)
            with self._lock:
                self._pos = (nlat, nlon, nalt)
                if rem <= 0.5:
                    self._cur_idx += 1
            time.sleep(self.tick)

# -------------------- 실제 MAVLink 경로 --------------------
def mav_connect():
    if mavutil is None:
        raise RuntimeError("pymavlink가 설치되어 있지 않습니다. (dry-run 모드로 실행하세요)")
    m = mavutil.mavlink_connection(CONN_STR, baud=BAUD)
    m.wait_heartbeat()
    print(f"[run] MAVLink connected: sys={m.target_system} comp={m.target_component}")
    return m

def upload_mission_from_points(m, latlonalt_list):
    ts, tc = m.target_system, m.target_component
    m.mav.mission_clear_all_send(ts, tc)
    total = len(latlonalt_list)
    m.mav.mission_count_send(ts, tc, total)
    sent = 0
    while sent < total:
        req = m.recv_match(type=["MISSION_REQUEST_INT","MISSION_REQUEST"], blocking=True, timeout=5)
        if req is None:
            raise TimeoutError("MISSION_REQUEST timeout")
        seq = req.seq
        lat, lon, alt = latlonalt_list[seq]
        m.mav.mission_item_int_send(
            ts, tc, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0,0,0,0,
            int(lat*1e7), int(lon*1e7), float(alt)
        )
        sent += 1
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
    print(f"[run] Mission upload ACK: {getattr(ack, 'type', 'None')}")

def set_mode(m, mode_name: str):
    try:
        m.set_mode_apm(mode_name)
        print(f"[run] Mode -> {mode_name}")
    except Exception as e:
        print(f"[run] Mode set failed: {e}")

def arm_and_takeoff(m, rel_alt: float):
    ts, tc = m.target_system, m.target_component
    m.mav.command_long_send(ts, tc, 400, 0, 1,0,0,0,0,0,0)  # ARM
    print("[run] ARM requested")
    set_mode(m, "GUIDED")
    m.mav.command_long_send(ts, tc, 22, 0, 0,0,0,0, 0,0, float(rel_alt))  # TAKEOFF
    print(f"[run] Takeoff to {rel_alt} m requested")

def telemetry_logger(m, stop_evt: threading.Event):
    LOG_DIR.mkdir(exist_ok=True)
    state_path = LOG_DIR / "live_state.json"
    track_path = LOG_DIR / "track.csv"
    if not track_path.exists():
        track_path.write_text("timestamp,lat,lon,alt\n", encoding="utf-8")
    while not stop_evt.is_set():
        try:
            msg = m.recv_match(type=["GLOBAL_POSITION_INT","GPS_RAW_INT"], blocking=False)
            if not msg:
                time.sleep(0.1); continue
            now = datetime.now(timezone.utc).isoformat()
            if msg.get_type() == "GLOBAL_POSITION_INT":
                lat = msg.lat/1e7; lon = msg.lon/1e7; alt = msg.relative_alt/1000.0
            elif msg.get_type() == "GPS_RAW_INT":
                lat = msg.lat/1e7; lon = msg.lon/1e7; alt = ""
            else:
                continue
            state = {"timestamp": now, "lat": lat, "lon": lon, "alt": alt if alt != "" else None}
            state_path.write_text(json.dumps(state), encoding="utf-8")
            with open(track_path, "a", encoding="utf-8") as f:
                f.write(f"{now},{lat},{lon},{alt}\n")
        except Exception:
            time.sleep(0.1)

# -------------------- 기본 체크 --------------------
def ensure_basics():
    LOG_DIR.mkdir(exist_ok=True)
    if not INPUT_APP.exists():
        print("[run] input.py가 없습니다.", file=sys.stderr); sys.exit(1)
    if not ROUTE_UPDATE_ADV.exists():
        print("[run] route_update_advanced.py가 없습니다.", file=sys.stderr); sys.exit(1)

# -------------------- 메인 --------------------
def main():
    ap = argparse.ArgumentParser(description="ADV-only mission runner with optional dry-run simulator")
    ap.add_argument("--dry-run", "--sim", action="store_true", help="MAVLink 없이 로컬 시뮬레이터로 테스트")
    ap.add_argument("--sim-speed", type=float, default=12.0, help="시뮬레이터 속도 m/s")
    ap.add_argument("--sim-tick", type=float, default=0.5, help="시뮬레이터 갱신 주기 s")
    args = ap.parse_args()

    ensure_basics()
    print(f"{INPUT_APP}")
    print("[run] 시작 (dry-run=%s)" % args.dry_run)

    # 1) input.py 실행 (8000이 비어있으면 띄우고, 아니면 이미 떠 있다고 보고 그냥 기다림)
    if is_port_free(8000):
        print("[run] input.py 실행 (홈/폴리곤 저장 UI @ 127.0.0.1:8000)")
        input_proc = subprocess.Popen([PY, str(INPUT_APP)], cwd=str(BASE_DIR))
    else:
        print("[run] 8000 포트가 이미 사용 중입니다. 기존 input.py가 떠 있다고 가정하고 진행합니다.")

    # 2) 초기 트리거: home + polygon 모두 준비 후 경로 계산
    wait_for_home_wp(timeout_sec=None)
    wait_for_polygon_txt(timeout_sec=None)

    print("[run] route_update_advanced.py 실행 (초기 경로 계산)")
    ret = subprocess.run([PY, str(ROUTE_UPDATE_ADV)],
                         cwd=str(BASE_DIR),
                         capture_output=True, text=True)
    if ret.stdout: print("[advanced 출력]\n" + ret.stdout, end="")
    if ret.stderr: print("[advanced 에러]\n" + ret.stderr, file=sys.stderr, end="")
    code = ret.returncode
    if code not in (0, NO_UPDATE_EXIT_CODE):
        print(f"[run] route_update_advanced.py 실패 (exit={code})", file=sys.stderr); sys.exit(1)

    # 3) 최신 route 확보
    route_path = latest_route_file()
    if not route_path:
        print("[run] 초기 route 파일을 찾지 못했습니다.", file=sys.stderr); sys.exit(1)
    print(f"[run] 초기 업로드 대상: {route_path.name}")
    latlonalt = parse_wp_latlonalt_list(route_path)
    if not latlonalt:
        print("[run] 미션 포인트가 없습니다.", file=sys.stderr); sys.exit(1)

    # 4) DRY-RUN or REAL
    stop_evt = threading.Event()
    ident_proc = None

    if args.dry_run:
        print("[run] DRY-RUN: MAVLink 없이 시뮬레이터로 진행합니다.")
        sim = RouteSimulator(speed_mps=args.sim_speed, tick=args.sim_tick, default_alt=REL_ALT)
        sim.set_route(latlonalt)
        sim.start()
        if IDENTIFIER.exists():
            print("[run] identifier.py 실행")
            ident_proc = subprocess.Popen([PY, str(IDENTIFIER)], cwd=str(BASE_DIR))
        else:
            print("[run] identifier.py 없음 → 미사용")

        last_prob = latest_prob_index()
        last_poly = latest_polygon_index()
        print(f"[run] 감시 시작: prob={last_prob}, polygon={last_poly}")
        stop = {"flag": False}
        def handle_sig(signum, frame):
            stop["flag"] = True
        for s in (signal.SIGINT, signal.SIGTERM):
            signal.signal(s, handle_sig)

        try:
            while not stop["flag"]:
                time.sleep(POLL_SEC)
                cur_prob = latest_prob_index()
                cur_poly = latest_polygon_index()

                if cur_prob > last_prob or cur_poly > last_poly:
                    why = []
                    if cur_prob > last_prob: why.append(f"prob{cur_prob}.csv")
                    if cur_poly > last_poly: why.append(f"polygon{cur_poly}.txt")
                    print(f"[run] 변경 감지: {', '.join(why)} → advanced 실행")

                    ret = subprocess.run([PY, str(ROUTE_UPDATE_ADV)],
                                         cwd=str(BASE_DIR),
                                         capture_output=True, text=True)
                    if ret.stdout: print("[advanced 출력]\n" + ret.stdout, end="")
                    if ret.stderr: print("[advanced 에러]\n" + ret.stderr, file=sys.stderr, end="")
                    code = ret.returncode
                    if code == NO_UPDATE_EXIT_CODE:
                        print("[run] advanced: 업데이트 없음 → run_mission 종료")
                        break
                    elif code != 0:
                        print(f"[run] advanced 실패 (exit={code})", file=sys.stderr)
                    else:
                        new_route = latest_route_file()
                        if new_route:
                            try:
                                new_pts = parse_wp_latlonalt_list(new_route)
                                if new_pts:
                                    print(f"[run] [SIM] 새 라우트 적용: {new_route.name} (pts={len(new_pts)})")
                                    sim.set_route(new_pts)
                                else:
                                    print("[run] [SIM] 새 미션 포인트가 비어 있음", file=sys.stderr)
                            except Exception as e:
                                print(f"[run] [SIM] 새 라우트 적용 실패: {e}", file=sys.stderr)
                        else:
                            print("[run] 최신 route 파일을 찾지 못함", file=sys.stderr)
                    last_prob = cur_prob
                    last_poly = cur_poly
        finally:
            if ident_proc and ident_proc.poll() is None:
                print("[run] identifier 종료 요청")
                ident_proc.terminate()
                try:
                    ident_proc.wait(3)
                except Exception:
                    ident_proc.kill()
            sim.stop()
            print("[run] 종료")
        return

    # === 실제 MAVLink 경로 ===
    if mavutil is None:
        print("[run] pymavlink가 설치되어 있지 않습니다. (dry-run 모드로 실행하세요)", file=sys.stderr)
        sys.exit(1)

    m = mav_connect()
    upload_mission_from_points(m, latlonalt)
    arm_and_takeoff(m, REL_ALT)
    set_mode(m, "AUTO")

    tlog = threading.Thread(target=telemetry_logger, args=(m, stop_evt), daemon=True)
    tlog.start()
    print("[run] telemetry logger started")

    if IDENTIFIER.exists():
        print("[run] identifier.py 실행")
        ident_proc = subprocess.Popen([PY, str(IDENTIFIER)], cwd=str(BASE_DIR))
    else:
        print("[run] identifier.py 없음 → 미사용")

    last_prob = latest_prob_index()
    last_poly = latest_polygon_index()
    print(f"[run] 감시 시작: prob={last_prob}, polygon={last_poly}")
    stop = {"flag": False}
    def handle_sig(signum, frame):
        stop["flag"] = True
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, handle_sig)

    try:
        while not stop["flag"]:
            time.sleep(POLL_SEC)
            cur_prob = latest_prob_index()
            cur_poly = latest_polygon_index()

            if cur_prob > last_prob or cur_poly > last_poly:
                why = []
                if cur_prob > last_prob: why.append(f"prob{cur_prob}.csv")
                if cur_poly > last_poly: why.append(f"polygon{cur_poly}.txt")
                print(f"[run] 변경 감지: {', '.join(why)} → advanced 실행")

                ret = subprocess.run([PY, str(ROUTE_UPDATE_ADV)],
                                     cwd=str(BASE_DIR),
                                     capture_output=True, text=True)
                if ret.stdout: print("[advanced 출력]\n" + ret.stdout, end="")
                if ret.stderr: print("[advanced 에러]\n" + ret.stderr, file=sys.stderr, end="")
                code = ret.returncode
                if code == NO_UPDATE_EXIT_CODE:
                    print("[run] advanced: 업데이트 없음 → run_mission 종료")
                    break
                elif code != 0:
                    print(f"[run] advanced 실패 (exit={code})", file=sys.stderr)
                else:
                    new_route = latest_route_file()
                    if new_route:
                        try:
                            new_pts = parse_wp_latlonalt_list(new_route)
                            if new_pts:
                                print(f"[run] 새 미션 업로드: {new_route.name} (pts={len(new_pts)})")
                                upload_mission_from_points(m, new_pts)
                                set_mode(m, "AUTO")
                            else:
                                print("[run] 새 미션 포인트가 비어 있음", file=sys.stderr)
                        except Exception as e:
                            print(f"[run] 새 미션 업로드 실패: {e}", file=sys.stderr)
                    else:
                        print("[run] 최신 route 파일을 찾지 못함", file=sys.stderr)
                last_prob = cur_prob
                last_poly = cur_poly
    finally:
        if ident_proc and ident_proc.poll() is None:
            print("[run] identifier 종료 요청")
            ident_proc.terminate()
            try:
                ident_proc.wait(3)
            except Exception:
                ident_proc.kill()
        stop_evt.set()
        print("[run] 종료")

if __name__ == "__main__":
    main()