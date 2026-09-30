"""電腦是大腦，機器人是軀體。

神經元來自 FlyWire FAFB v783（Schlegel 等人 2024 的註解表），
突觸來自同一版的 proofread_connections。只載入朝向與轉向這一小群，
在電腦上算活動，再把左右差收成前進或轉向，送給 ESP32。
"""

import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import upload_and_run, read_until

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
        self.turn_step = 35.0
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

    def decide(self):
        rates = self.activity()
        vx = vy = 0.0
        for i, n in enumerate(self.neurons):
            if n["role"] != "compass":
                continue
            vx += rates[i] * math.cos(n["angle"])
            vy += rates[i] * math.sin(n["angle"])
        estimate = math.degrees(math.atan2(vy, vx)) % 360.0
        error = (self.goal - estimate + 180.0) % 360.0 - 180.0
        left = sum(rates[i] for i in self.dna if self.neurons[i]["side"] == "left")
        right = sum(rates[i] for i in self.dna if self.neurons[i]["side"] == "right")
        if error > 30:
            command = "right"
            self.heading = (self.heading + self.turn_step) % 360.0
        elif error < -30:
            command = "left"
            self.heading = (self.heading - self.turn_step) % 360.0
        else:
            command = "forward"
        return command, estimate, error, left, right


def send(ser, command):
    ser.write((command + "\n").encode("utf-8"))
    reply = read_until(ser, "ok", 8)
    return "ok" in reply


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
    brain = SteeringBrain(circuit)
    brain.goal = 90.0
    ser = upload_and_run()
    boot = read_until(ser, "READY", 8)
    print(boot[-160:])
    if "READY" not in boot:
        ser.close()
        raise SystemExit("not ready")
    for step in range(6):
        command, estimate, error, left, right = brain.decide()
        print(
            "step",
            step + 1,
            "EPG",
            round(estimate),
            "error",
            round(error),
            "DNa02 L",
            round(left, 4),
            "R",
            round(right, 4),
            "->",
            command,
        )
        if not send(ser, command):
            print("NO_REPLY")
            break
    send(ser, "stop")
    ser.close()
    print("DONE")


if __name__ == "__main__":
    main()
