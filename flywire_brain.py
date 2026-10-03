"""The computer is the brain. The robot is the body.

Neurons are the FlyWire FAFB v783 heading set (Schlegel et al., Nature 2024).
Synapses are the matching proofread connection table. Only the heading and
steering group is loaded. Activity is computed on the computer, then sent to
the ESP32 as a straight step or a curved step of the same gait.
"""

import csv
import json
import math
import os
import random
import socket
import sys
import threading
import time

DATA = r"C:\Users\user\Desktop\bot1\data\flywire"
ANNOTATIONS = os.path.join(DATA, "neuron_annotations.tsv")
SYNAPSES = os.path.join(DATA, "proofread_connections_783.feather")
CIRCUIT = os.path.join(DATA, "steering_circuit.json")

KEEP = {
    "EPG": "compass",
    "EPGt": "compass",
    "PEG": "update",
    "PEN_a(PEN1)": "update",
    "PEN_b(PEN2)": "update",
    "PFL1": "steer",
    "PFL2": "steer",
    "PFL3": "steer",
    "DNa01": "descend",
    "DNa02": "descend",
}
SYNAPSE_BYTES = 852022274


def angle_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def load_neurons():
    neurons = []
    with open(ANNOTATIONS, encoding="utf-8") as handle:
        for row in csv.DictReader(handle, delimiter="\t"):
            role = KEEP.get(row["cell_type"])
            if role is None:
                continue
            neurons.append(
                {
                    "root_id": row["root_id"],
                    "cell_type": row["cell_type"],
                    "role": role,
                    "side": row["side"],
                    "nt": (row["top_nt"] or "").lower(),
                    "x": float(row["pos_x"] or 0),
                    "y": float(row["pos_y"] or 0),
                    "z": float(row["pos_z"] or 0),
                }
            )
    compass = [n for n in neurons if n["role"] == "compass"]
    cx = sum(n["x"] for n in compass) / len(compass)
    cy = sum(n["y"] for n in compass) / len(compass)
    for n in neurons:
        n["angle"] = math.atan2(n["y"] - cy, n["x"] - cx)
    return neurons


def load_edges(neurons):
    import pyarrow as pa
    import pyarrow.compute as pc
    import pyarrow.feather as feather

    ids = {n["root_id"]: i for i, n in enumerate(neurons)}
    id_values = pa.array([int(n["root_id"]) for n in neurons])
    table = feather.read_table(
        SYNAPSES,
        columns=["pre_pt_root_id", "post_pt_root_id", "syn_count", "gaba_avg", "ach_avg", "glut_avg"],
    )
    keep = pc.and_(
        pc.is_in(table.column("pre_pt_root_id"), value_set=id_values),
        pc.is_in(table.column("post_pt_root_id"), value_set=id_values),
    )
    table = table.filter(keep)
    weights = {}
    for a, b, count, gaba, ach, glut in zip(
        table.column("pre_pt_root_id").to_pylist(),
        table.column("post_pt_root_id").to_pylist(),
        table.column("syn_count").to_pylist(),
        table.column("gaba_avg").to_pylist(),
        table.column("ach_avg").to_pylist(),
        table.column("glut_avg").to_pylist(),
    ):
        i = ids.get(str(a))
        j = ids.get(str(b))
        if i is None or j is None or i == j:
            continue
        gaba = gaba or 0.0
        ach = ach or 0.0
        glut = glut or 0.0
        sign = -1.0 if max(gaba, glut) >= ach and max(gaba, glut) > 0 else 1.0
        weights[(i, j)] = weights.get((i, j), 0.0) + float(count) * sign
    return [
        {"pre": i, "post": j, "weight": round(w, 2)}
        for (i, j), w in weights.items()
        if abs(w) >= 5
    ]


def build_circuit():
    neurons = load_neurons()
    edges = load_edges(neurons)
    circuit = {
        "source": "FlyWire FAFB v783",
        "annotations": "Schlegel et al. Nature 2024, Supplemental file 1",
        "synapses": "FlyWire proofread_connections_783",
        "neurons": neurons,
        "edges": edges,
    }
    with open(CIRCUIT, "w", encoding="utf-8") as handle:
        json.dump(circuit, handle)
    return circuit


def wait_for_synapses():
    while True:
        if os.path.exists(SYNAPSES) and os.path.getsize(SYNAPSES) >= SYNAPSE_BYTES:
            return
        time.sleep(5)


class SteeringBrain:
    def __init__(self, circuit):
        self.neurons = circuit["neurons"]
        self.n = len(self.neurons)
        self.incoming = [[] for _ in range(self.n)]
        row_sum = [0.0] * self.n
        for edge in circuit["edges"]:
            self.incoming[edge["post"]].append((edge["pre"], edge["weight"]))
            row_sum[edge["post"]] += abs(edge["weight"])
        scale = max(row_sum) or 1.0
        self.incoming = [
            [(pre, weight / scale) for pre, weight in row]
            for row in self.incoming
        ]
        self.heading = 0.0
        self.goal = 90.0
        self.turn_step = 7.0
        self.threshold = 8.0
        self.dna = [
            i
            for i, n in enumerate(self.neurons)
            if n["cell_type"] == "DNa02"
        ]

    def activity(self):
        heading = math.radians(self.heading)
        drive = [0.0] * self.n
        for i, n in enumerate(self.neurons):
            if n["role"] == "compass":
                delta = angle_diff(n["angle"], heading)
                drive[i] = math.exp(-0.5 * (delta / 0.45) ** 2)
        rates = [0.0] * self.n
        for _ in range(18):
            nxt = []
            for i in range(self.n):
                total = drive[i]
                for pre, weight in self.incoming[i]:
                    total += weight * rates[pre]
                nxt.append(total if total > 0.0 else 0.0)
            rates = [0.5 * old + 0.5 * new for old, new in zip(rates, nxt)]
        return rates

    def sense(self):
        rates = self.activity()
        vx = vy = 0.0
        for i, n in enumerate(self.neurons):
            if n["role"] != "compass":
                continue
            vx += rates[i] * math.cos(n["angle"])
            vy += rates[i] * math.sin(n["angle"])
        estimate = math.degrees(math.atan2(vy, vx)) % 360.0
        left = sum(rates[i] for i in self.dna if self.neurons[i]["side"] == "left")
        right = sum(rates[i] for i in self.dna if self.neurons[i]["side"] == "right")
        return estimate, left, right

    def decide(self):
        estimate, left, right = self.sense()
        error = (self.goal - estimate + 180.0) % 360.0 - 180.0
        if error > self.threshold:
            command = "right"
            self.heading = (self.heading + self.turn_step) % 360.0
        elif error < -self.threshold:
            command = "left"
            self.heading = (self.heading - self.turn_step) % 360.0
        else:
            command = "forward"
        return command, estimate, error, left, right


IP_FILE = r"C:\Users\user\Desktop\bot1\data\robot_ip.txt"
PORT = 8765
HIP = 28
KNEE = 36
# Front-right, front-left, rear-right, rear-left. The fly's tripod, reduced to a quadruped, steps on the diagonal: front-left with rear-right, then front-right with rear-left.
LEGS = (
    (0, 1, 2, -1, "right"),
    (1, -1, 3, -1, "left"),
    (4, -1, 6, 1, "right"),
    (5, -1, 7, 1, "left"),
)
BASE_PHASE = (0.5, 0.0, 0.0, 0.5)


def _smooth(u):
    return 0.5 - 0.5 * math.cos(math.pi * u)


def _leg(phase, hip_sign, knee_sign, hip_dir, knee):
    phase = phase % 1.0
    if phase < 0.18:
        u = _smooth(phase / 0.18)
        knee_u = u
        hip_u = -1.0
    elif phase < 0.42:
        u = _smooth((phase - 0.18) / 0.24)
        knee_u = 1.0
        hip_u = -1.0 + 2.0 * u
    elif phase < 0.56:
        u = _smooth((phase - 0.42) / 0.14)
        knee_u = 1.0 - u
        hip_u = 1.0
    else:
        u = _smooth((phase - 0.56) / 0.44)
        knee_u = 0.0
        hip_u = 1.0 - 2.0 * u
    hip = 90 + hip_sign * hip_dir * HIP * hip_u
    knee_angle = 90 + knee_sign * knee * knee_u
    return hip, knee_angle


def _hip_dirs(error):
    # A right turn lengthens the left step and shortens the right step, the same side bias DNa02 uses, with a cap so the body does not slide sideways.
    mix = min(0.4, abs(error) / 120.0)
    weaken = "right" if error > 0 else "left"
    dirs = []
    for *_idx, side in LEGS:
        dirs.append(1.0 - mix if side == weaken else 1.0)
    return dirs


def _pose(phase, hip_dirs, knee):
    pose = [90] * 8
    nudge = hip_dirs[0] - hip_dirs[1]
    for n, (hip_i, hip_s, knee_i, knee_s, side) in enumerate(LEGS):
        shift = -0.05 * nudge if side == "right" else 0.05 * nudge
        hip, knee_angle = _leg(
            phase + BASE_PHASE[n] + shift,
            hip_s,
            knee_s,
            hip_dirs[n],
            knee,
        )
        pose[hip_i] = max(0, min(180, int(round(hip))))
        pose[knee_i] = max(0, min(180, int(round(knee_angle))))
    return pose


def once(ip, command):
    sock = socket.create_connection((ip, PORT), 8)
    sock.settimeout(12)
    sock.sendall((command + "\n").encode("utf-8"))
    reply = sock.recv(32).decode("utf-8", "replace").strip()
    sock.close()
    return reply


def _send_pose(ip, ms, pose, stop):
    if stop is not None and stop.is_set():
        return "stop"
    command = "m %d %s" % (ms, " ".join(str(v) for v in pose))
    reply = once(ip, command)
    if stop is not None and stop.is_set():
        return "stop"
    return reply


RUN_MS = 300
JOG_MS = 740
TURN_ERROR = 40.0


def stride(ip, error, knee, stop=None, period=RUN_MS):
    if stop is not None and stop.is_set():
        return "stop"
    reply = once(ip, "bias %d %d" % (int(round(error)), period))
    if stop is not None and stop.is_set():
        return "stop"
    return reply


class Wander:
    # Jog, run, and turn each start on their own. Nothing waits for a fixed step count.
    def __init__(self):
        self.turn_left = 0
        self.turn_sign = 1.0
        self.pace_left = 0
        self.pace = RUN_MS
        self.pace_name = "run"

    def begin_turn(self):
        self.turn_left = random.randint(3, 10)
        self.turn_sign = random.choice((-1.0, 1.0))
        side = "right" if self.turn_sign > 0 else "left"
        print("turn", self.turn_left, side)

    def begin_pace(self):
        if random.random() < 0.5:
            self.pace = JOG_MS
            self.pace_name = "jog"
            self.pace_left = random.randint(2, 6)
        else:
            self.pace = RUN_MS
            self.pace_name = "run"
            self.pace_left = random.randint(2, 8)
        print(self.pace_name, self.pace_left, self.pace)

    def step(self, brain):
        if self.turn_left <= 0 and random.random() < 0.18:
            self.begin_turn()
        if self.turn_left > 0:
            error = TURN_ERROR * self.turn_sign
            brain.heading = (brain.heading + brain.turn_step * self.turn_sign) % 360.0
            self.turn_left -= 1
        else:
            error = 0.0
        if self.pace_left <= 0:
            self.begin_pace()
        self.pace_left -= 1
        return error, self.pace


def _knee_now(brain):
    peak = max(
        (brain.activity()[i] for i, n in enumerate(brain.neurons) if n["role"] == "compass"),
        default=1.0,
    )
    return KNEE * min(1.0, max(0.7, peak))


def _watch_stop(stop):
    while not stop.is_set():
        line = sys.stdin.readline()
        if not line:
            stop.set()
            return
        if line.strip().lower() == "stop":
            stop.set()
            return


def live(ip, brain):
    stop = threading.Event()
    threading.Thread(target=_watch_stop, args=(stop,), daemon=True).start()
    print("Jog, run, and turns start on their own. A turn keeps the same gait for 3 to 10 steps. Type stop and press Enter to stand and halt.")
    brain.heading = 0.0
    brain.goal = 0.0
    wander = Wander()
    while not stop.is_set():
        estimate, left, right = brain.sense()
        error, period = wander.step(brain)
        print(
            "walk",
            wander.pace_name,
            period,
            "EPG",
            round(estimate),
            "curve",
            round(error),
            "DNa02 L",
            round(left, 4),
            "R",
            round(right, 4),
        )
        stride(ip, error, _knee_now(brain), stop, period)
    print("stand", once(ip, "stop"))


def main():
    if not os.path.exists(CIRCUIT):
        print("waiting for FlyWire synapse table")
        wait_for_synapses()
        circuit = build_circuit()
    else:
        with open(CIRCUIT, encoding="utf-8") as handle:
            circuit = json.load(handle)
    print(
        "loaded",
        len(circuit["neurons"]),
        "neurons",
        len(circuit["edges"]),
        "synapses",
    )
    try:
        ip = open(IP_FILE, encoding="utf-8").read().strip()
    except OSError:
        raise SystemExit("missing data/robot_ip.txt")
    brain = SteeringBrain(circuit)
    brain.heading = 0.0
    brain.goal = 0.0
    brain.turn_step = 7.0
    if len(sys.argv) > 1:
        steps = int(sys.argv[1])
        wander = Wander()
        for step in range(steps):
            estimate, left, right = brain.sense()
            error, period = wander.step(brain)
            print(
                "step",
                step + 1,
                wander.pace_name,
                period,
                "EPG",
                round(estimate),
                "curve",
                round(error),
                "DNa02 L",
                round(left, 4),
                "R",
                round(right, 4),
                "knee",
                round(_knee_now(brain)),
            )
            reply = stride(ip, error, _knee_now(brain), None, period)
            print(" ", reply)
            if reply != "ok":
                break
        print("stand", once(ip, "stop"))
        print("DONE")
        return
    live(ip, brain)


if __name__ == "__main__":
    main()
