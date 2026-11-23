import json
import numpy as np

def load_json(ref_dir, name):
    return json.loads((ref_dir / name).read_text())

def load_plant(ref_dir):
    d = load_json(ref_dir, "plant_nominal.json")
    return {
        "A":   np.array(d["A"], dtype=float),
        "B_u": np.array(d["B_u"], dtype=float).reshape(-1, 1),
        "B_w": np.array(d["B_w"], dtype=float).reshape(-1, 1),
        "C":   np.array(d["C"], dtype=float),
        "D_u": np.array(d["D_u"], dtype=float).reshape(-1, 1),
        "D_w": np.array(d["D_w"], dtype=float).reshape(-1, 1),
    }

def load_sim(ref_dir, scenario, system):
    path = ref_dir / f"sim_{scenario}_{system}.csv"
    return np.genfromtxt(path, delimiter=",", names=True)   # columns by name: data["t"]