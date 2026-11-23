from quarter_car.loader import *

# This test suite is responsible for the testing of correct functioning of the fixtures in conftest.py
def test_plant_shape(plant):
    assert plant["A"].shape == (4,4)

def test_K_shape(K):
    assert K.shape==(1,4)

def test_scen_keys(scen_conf):
    keys = scen_conf["scenarios"].keys()
    assert set(keys) == {'ramp', 'bump', 'rough'}

def test_rough_road(rough_scenario_comp):
    freq = rough_scenario_comp["n_cycles_per_m"]
    amp = rough_scenario_comp["amplitude_m"]
    phase = rough_scenario_comp["phase_rad"]
    assert all(np.array(amp)>0)
    assert len(freq)==len(phase)==len(amp)

def test_feedback_eigs(plant,K):
    A = plant["A"]
    B_u = plant["B_u"]

    feedback = A - B_u @ K
    eigs = np.linalg.eigvals(feedback)
    assert all(np.real(np.array(eigs))<0)