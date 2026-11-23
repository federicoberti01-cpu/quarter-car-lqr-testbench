from pathlib import Path
import pytest
import numpy as np
from quarter_car.loader import load_json,load_plant

REPO_ROOT = Path(__file__).resolve().parents[1]

@pytest.fixture(scope="session")
def ref_dir() -> Path:
    # Retrieves the reference directory containing .json exports from Matlab
    path = REPO_ROOT / "reference"
    assert path.is_dir(), f"reference folder not found: {path}"
    return path

@pytest.fixture(scope="session")
def plant(ref_dir) -> dict:
    # Loads plant
    return load_plant(ref_dir)

@pytest.fixture(scope="session")
def K(ref_dir) -> np.ndarray:
    # Loads K feedback matrix
    cont = load_json(ref_dir, "controller.json")
    return np.array(cont["K"], dtype = float).reshape(1,-1)

@pytest.fixture(scope="session")
def scen_conf(ref_dir) -> dict:
    # Retrieves scenario configuration file
    return load_json(ref_dir, "scenarios.json")

@pytest.fixture(scope="session")
def rough_scenario_comp(ref_dir) -> dict:
    # Retrieves rough scenario config file
    return load_json(ref_dir, "rough_road_components.json")
