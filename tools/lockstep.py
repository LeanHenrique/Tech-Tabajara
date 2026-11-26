#!/usr/bin/env python3
"""Partidas aceleradas e reprodutíveis com o simulador do kit, sem ROS.

Roda o `World` do kit (psr_sim: mesma física, mesmo juiz) no mesmo processo que os nós dos jogos. O ROS e a
ponte (hero_bridge.py) são trocados por um barramento em processo com relógio simulado:

  * o simulador avança em passos de 0,02 s (50 Hz), como o `Hub` do kit;
  * a ponte republica o estado a cada 2 passos (25 Hz), com os mesmos tópicos e campos do hero_bridge.py;
  * cada nó roda o próprio `step()` no ritmo do `rospy.Rate` dele (20 Hz), com uma fase própria;
  * um comando publicado vale a partir do passo seguinte do simulador, como chegaria pela ponte.

Os arquivos dos jogos são carregados sem alteração. Cada nó tem o seu `~param`, o seu log e o seu relógio
(`rospy.get_time()` devolve o tempo simulado). Uma exceção num callback é registrada no log do nó como erro e o
nó continua, como no rospy.

Diferenças em relação à máquina do torneio: não há latência de rede nem variação do escalonador, e as sementes
são fixas. Cada placar é uma amostra, não uma taxa.

Uso:
  python tools/lockstep.py --kit "Trabalho Prático 5/sim" --utils "Trabalho Prático 5/ros/psr_hero/src" \
      --plan tools/planos/ctf.json --out exercicios/registros/final [--only NOME] [--jobs N]

O Python precisa ser o do ambiente do kit (o que tem o Genesis), por exemplo "Trabalho Prático 5/.venv".
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import multiprocessing
import os
import sys
import time
import traceback
import types
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATE_EVERY = 2  # passos de 0,02 s entre publicações da ponte (25 Hz)
FRAME_EVERY = 5  # passos entre quadros gravados (10 Hz)


# --- mensagens (só os campos que a ponte preenche) ------------------------------------------------------------------

class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _vec():
    return _Obj(x=0.0, y=0.0, z=0.0)


class Twist:
    def __init__(self):
        self.linear, self.angular = _vec(), _vec()


class Odometry:
    def __init__(self):
        self.header = _Obj(stamp=0.0, frame_id="")
        self.child_frame_id = ""
        self.pose = _Obj(pose=_Obj(position=_vec(), orientation=_Obj(x=0.0, y=0.0, z=0.0, w=1.0)))
        self.twist = _Obj(twist=_Obj(linear=_vec(), angular=_vec()))


class PointStamped:
    def __init__(self):
        self.header = _Obj(stamp=0.0, frame_id="")
        self.point = _vec()


class Bool:
    def __init__(self, data=False):
        self.data = data


class String:
    def __init__(self, data=""):
        self.data = data


class LaserScan:
    def __init__(self):
        self.ranges = []


def _install_messages():
    for name, attrs in {
        "geometry_msgs": {}, "geometry_msgs.msg": {"Twist": Twist, "PointStamped": PointStamped},
        "nav_msgs": {}, "nav_msgs.msg": {"Odometry": Odometry},
        "std_msgs": {}, "std_msgs.msg": {"Bool": Bool, "String": String},
        "sensor_msgs": {}, "sensor_msgs.msg": {"LaserScan": LaserScan},
    }.items():
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m


# --- barramento e rospy simulado ------------------------------------------------------------------------------------

class Node:
    def __init__(self, name, params, path, phase):
        self.name, self.params, self.path, self.phase = name, params, path, phase
        self.log = []  # (t, nível, texto)
        self.errors = 0
        self.throttle = {}
        self.rate_hz = None
        self.obj = None


class Bus:
    def __init__(self):
        self.subs = {}  # tópico -> [(nó, callback, args)]
        self.current = None
        self.t = 0.0

    def publish(self, topic, msg):
        for node, cb, args in list(self.subs.get(topic, [])):
            self.call(node, cb, msg, *args)

    def call(self, node, fn, *args):
        prev, self.current = self.current, node
        try:
            return fn(*args)
        except Exception:  # o rospy registra a exceção do callback e o nó continua
            node.errors += 1
            node.log.append((self.t, "ERROR", traceback.format_exc().strip().replace("\n", " | ")))
            return None
        finally:
            self.current = prev


class FakeRospy(types.ModuleType):
    def __init__(self, bus):
        super().__init__("rospy")
        self.bus = bus
        self.on_cmd = None  # (tópico, msg) de um nó -> simulador

    # parâmetros e relógio
    def init_node(self, *a, **k):
        pass

    def get_param(self, name, default=None):
        node = self.bus.current
        return node.params.get(name.lstrip("~"), default) if node else default

    def get_time(self):
        return self.bus.t

    def is_shutdown(self):
        return False

    def Rate(self, hz):
        if self.bus.current is not None:
            self.bus.current.rate_hz = float(hz)
        return _Obj(sleep=lambda: None)

    # comunicação
    def Subscriber(self, topic, cls, cb, callback_args=None, queue_size=None):
        args = () if callback_args is None else (callback_args,)
        self.bus.subs.setdefault(topic, []).append((self.bus.current, cb, args))

    def Publisher(self, topic, cls, queue_size=None):
        bus, outer, owner = self.bus, self, self.bus.current

        def publish(msg):
            if topic.endswith("/cmd_vel") and outer.on_cmd is not None:
                outer.on_cmd(owner, topic, msg)
            bus.publish(topic, msg)
        return _Obj(publish=publish)

    # logs
    def _log(self, level, text, *args):
        node = self.bus.current
        if node is None:
            return
        if args:
            text = text % args
        node.log.append((self.bus.t, level, str(text)))
        if level == "ERROR":
            node.errors += 1

    def _throttled(self, level, period, text, *args):
        node = self.bus.current
        if node is None:
            return
        key = (level, str(text))
        last = node.throttle.get(key)
        if last is not None and self.bus.t - last < period:
            return
        node.throttle[key] = self.bus.t
        self._log(level, text, *args)

    def logdebug(self, *a): pass
    def loginfo(self, text, *a): self._log("INFO", text, *a)
    def logwarn(self, text, *a): self._log("WARN", text, *a)
    def logerr(self, text, *a): self._log("ERROR", text, *a)
    def loginfo_throttle(self, period, text, *a): self._throttled("INFO", period, text, *a)
    def logwarn_throttle(self, period, text, *a): self._throttled("WARN", period, text, *a)
    def logerr_throttle(self, period, text, *a): self._throttled("ERROR", period, text, *a)


# --- ponte -----------------------------------------------------------------------------------------------------------

def _odom(x, y, yaw, v, w):
    m = Odometry()
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    m.twist.twist.linear.x, m.twist.twist.angular.z = v, w
    return m


def bridge_publish(bus, st):
    """O que o hero_bridge.py publica a cada estado (25 Hz)."""
    for r in st["robots"]:
        bus.publish("/hero_%d/odom" % r["id"], _odom(r["x"], r["y"], r["yaw"], r["v"], r["w"]))
    puck = st.get("puck")
    if puck:
        m = _odom(puck["x"], puck["y"], 0.0, 0.0, 0.0)
        m.twist.twist.linear.x, m.twist.twist.linear.y = puck["vx"], puck["vy"]
        bus.publish("/ball/odom", m)
    for mk in st.get("markers", []):
        if mk.get("kind") == "flag":
            p = PointStamped()
            p.point.x, p.point.y = mk["x"], mk["y"]
            bus.publish("/flag/" + mk["team"], p)
    for i, carrying in enumerate(st.get("game", {}).get("carrying", [])):
        bus.publish("/hero_%d/has_flag" % i, Bool(data=bool(carrying)))
    game = dict(st.get("game", {}))
    game["t"] = round(st["t"], 2)
    game["message"] = st.get("message", "")
    bus.publish("/game/state", String(data=json.dumps(game)))


# --- uma série --------------------------------------------------------------------------------------------------------

def node_specs(series):
    """Nós da série, como os launch files do kit (exercicios/launch/*.launch) os criam."""
    sc, games = series["scenario"], series["files"]
    specs = []
    if sc == "sumo":
        for i in (0, 1):
            f = games.get("robot%d" % i)
            if f:
                specs.append(("sumo_%d" % i, f, {"robot": i, "opponent": 1 - i}))
    elif sc == "ctf":
        for team, ids, opp in (("blue", (0, 1), [2, 3]), ("red", (2, 3), [0, 1])):
            f = games.get(team)
            if f:
                for i, role in zip(ids, ("attacker", "defender")):
                    specs.append(("ctf_%d" % i, f, {"robot": i, "team": team, "role": role, "opponents": opp}))
    elif sc == "soccer":
        for team in ("blue", "red"):
            f = games.get(team)
            if f:
                specs.append(("soccer_" + team, f, {"team": team}))
    return specs


def load_node(rospy, bus, spec, phase, k):
    name, path, params = spec
    node = Node(name, params, path, phase)
    full = ROOT / path
    mod_name = "jogo_%s_%d" % (name, k)
    s = importlib.util.spec_from_file_location(mod_name, full)
    mod = importlib.util.module_from_spec(s)
    sys.modules[mod_name] = mod
    bus.current = node
    try:
        s.loader.exec_module(mod)
        cls = [v for v in vars(mod).values() if isinstance(v, type) and v.__module__ == mod_name and hasattr(v, "step")]
        if len(cls) != 1:
            raise RuntimeError("esperava uma classe de nó com step() em %s, achei %d" % (path, len(cls)))
        node.obj = cls[0]()
        # o run() publica (0, 0) e cria o Rate antes do laço; repetimos só essas duas coisas
        hz = getattr(mod, "RATE_HZ", 20.0)
        rospy.Rate(hz)
        node.rate_hz = node.rate_hz or float(hz)
    finally:
        bus.current = None
    return node


def score_of(sc, game):
    if sc == "sumo":
        return {"wins": list(game["wins"]), "draws": game["draws"]}
    if sc == "ctf":
        return {"score": list(game["score"]), "tags": list(game["tags"])}
    return {"score": [game["blue"], game["red"]]}


def run_series(series, kit, utils, out_dir):
    sys.path[:0] = [str(kit), str(utils)]
    _install_messages()
    bus = Bus()
    rospy = FakeRospy(bus)
    sys.modules["rospy"] = rospy

    from psr_sim.scenarios import SCENARIOS
    from psr_sim.world import World

    sc_name = series["scenario"]
    n = {"sumo": 2, "ctf": 4, "soccer": 6}[sc_name]
    scenario = SCENARIOS[sc_name](n, series["seed"])
    world = World(scenario, backend="cpu", viewer=False)

    def on_cmd(owner, topic, msg):
        i = int(topic.split("/")[1].split("_")[1])
        mine = owner is not None and i in owner.allowed
        if not mine:  # o nó só pode publicar nos robôs do próprio time
            owner.errors += 1
            owner.log.append((bus.t, "ERROR", "publicou em %s, que não é do próprio time" % topic))
        pending.append((i, float(msg.linear.x), float(msg.angular.z)))

    rospy.on_cmd = on_cmd
    pending = []

    nodes = []
    for k, spec in enumerate(node_specs(series)):
        phase = (0.013 * (k + 1) + 0.007 * series["seed"]) % 0.05
        node = load_node(rospy, bus, spec, phase, k)
        p = spec[2]
        if sc_name == "sumo":
            node.allowed = {p["robot"]}
        elif sc_name == "ctf":
            node.allowed = {p["robot"]}
        else:
            node.allowed = set(range(0, 3) if p["team"] == "blue" else range(3, 6))
        node.next_t = phase
        nodes.append(node)

    frames, events = [], []
    last_pub = None
    duration = series.get("duration")
    rounds = series.get("rounds")
    t0 = time.monotonic()
    step = 0
    prev_held = None
    while True:
        for i, lin, ang in pending:
            world.set_cmd(i, lin, ang)
        pending.clear()

        st = world.tick()
        st["goals"] = world.scenario.goals()
        st["markers"] = world.scenario.markers()
        bus.t = world.t
        game = st["game"]
        step += 1

        if step % STATE_EVERY == 0:
            bridge_publish(bus, st)
        for node in nodes:
            while node.next_t <= world.t + 1e-9:
                bus.call(node, node.obj.step)
                node.next_t += 1.0 / node.rate_hz

        pub = score_of(sc_name, game)
        if last_pub is not None and pub != last_pub:
            ev = {"t": round(world.t, 2), "message": st.get("message", ""), **pub}
            if sc_name == "sumo":
                ev["round_time"] = game.get("round_time")
            events.append(ev)
        last_pub = pub
        if sc_name == "ctf":
            held = game.get("held")
            if prev_held is not None:
                for i, (a, b) in enumerate(zip(prev_held, held)):
                    if b and not a:
                        events.append({"t": round(world.t, 2), "held": i, "score": list(game["score"])})
            prev_held = held

        if step % FRAME_EVERY == 0:
            fr = {"t": round(world.t, 2),
                  "r": [[round(r["x"], 4), round(r["y"], 4), round(r["yaw"], 3), int(r["held"])] for r in st["robots"]],
                  "m": st.get("message", "")}
            if st.get("puck"):
                fr["b"] = [round(st["puck"]["x"], 4), round(st["puck"]["y"], 4)]
            if sc_name == "ctf":
                fr["f"] = [[round(mk["x"], 4), round(mk["y"], 4)] for mk in st["markers"] if mk.get("kind") == "flag"]
            fr["g"] = pub
            frames.append(fr)

        if duration is not None and world.t >= duration:
            break
        if rounds is not None and sum(game["wins"]) + game["draws"] >= rounds and world.pending is None:
            break
        if rounds is not None and world.t > rounds * 60:  # proteção: nunca deveria chegar aqui
            break

    wall = time.monotonic() - t0
    world.close()
    errors = {node.name: node.errors for node in nodes}
    result = {
        "name": series["name"], "scenario": sc_name, "files": series["files"], "seed": series["seed"],
        "ours": series.get("ours"), "opponent": series.get("opponent"),
        "sim_time": round(world.t, 2), "wall_time": round(wall, 1), "final": last_pub,
        "errors": errors, "error_total": sum(errors.values()), "events": events,
        "driver": {"state_hz": 50 / STATE_EVERY, "node_hz": {nd.name: nd.rate_hz for nd in nodes},
                   "phase_s": {nd.name: round(nd.phase, 3) for nd in nodes}},
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / (series["name"] + ".json")).write_text(json.dumps(result, ensure_ascii=False, indent=1))
    (out_dir / (series["name"] + "_frames.json")).write_text(json.dumps({"scenario": sc_name, "frames": frames}))
    with open(out_dir / (series["name"] + ".log"), "w") as f:
        for node in nodes:
            for t, level, text in node.log:
                f.write("%8.2f %-5s [%s] %s\n" % (t, level, node.name, text))
    return result


def _worker(series, kit, utils, out_dir):
    return run_series(series, Path(kit), Path(utils), Path(out_dir))


def write_summary(out_dir):
    rows = []
    for p in sorted(out_dir.glob("*.json")):
        if p.name.endswith("_frames.json"):
            continue
        r = json.loads(p.read_text())
        rows.append([r["name"], r["scenario"], r.get("ours"), r.get("opponent"), r["seed"], r["sim_time"],
                     json.dumps(r["final"]), r["error_total"]])
    with open(out_dir / "resumo.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["serie", "jogo", "nosso_lado", "adversario", "semente", "tempo_simulado_s", "placar_final", "erros"])
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--kit", required=True, help="pasta sim/ do kit (contém psr_sim)")
    ap.add_argument("--utils", required=True, help="pasta ros/psr_hero/src do kit (contém psr_hero.utils)")
    ap.add_argument("--plan", required=True, nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", nargs="*", default=None, help="só as séries com esses nomes")
    ap.add_argument("--jobs", type=int, default=1)
    a = ap.parse_args()

    series = []
    for plan in a.plan:
        series += json.loads(Path(plan).read_text())["series"]
    if a.only:
        series = [s for s in series if s["name"] in a.only]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    # um processo novo por série: o Genesis só pode ser inicializado uma vez por processo
    ctx = multiprocessing.get_context("spawn")
    with ProcessPoolExecutor(max_workers=a.jobs, mp_context=ctx, max_tasks_per_child=1) as ex:
        futs = {ex.submit(_worker, s, a.kit, a.utils, str(out)): s["name"] for s in series}
        for fut in as_completed(futs):
            name = futs[fut]
            try:
                r = fut.result()
                print("%-32s %8.1f s simulados em %7.1f s  final=%s  erros=%d" %
                      (name, r["sim_time"], r["wall_time"], json.dumps(r["final"]), r["error_total"]), flush=True)
            except Exception:
                print("%-32s FALHOU NO DRIVER\n%s" % (name, traceback.format_exc()), flush=True)
    write_summary(out)


if __name__ == "__main__":
    main()
