from quarter_car.loader import load_sim, load_json
from quarter_car.scenarios import time_vector, make_road
import numpy as np


# This test suite ensures road inputs are correctly transferred between Matlab and Python
def test_time_vector_length():
  #Tests that the generated time vectors are consistent with Matlab for length
  t_vector = time_vector(10, 1e-3)
  assert len(t_vector) == 10001

def test_time_start_and_end():
  # Tests that the generated time vectors are consistent with Matlab for start and end
  t_vector = time_vector(10, 1e-3)
  np.testing.assert_allclose([t_vector[0], t_vector[-1]], [0,10], atol = 1e-6)

def test_time_matlab_matching(ref_dir):
  # Tests that the time vector from matlab is consistent with the python generated one
  matlab_sim = load_sim(ref_dir,"bump","passive")

  py_sim_t = time_vector(10, 1e-3)
  assert len(py_sim_t) == len(matlab_sim["t"])
  assert np.allclose(py_sim_t, matlab_sim["t"], rtol = 0, atol = 1e-6)

def test_ramp_matlab_matching(ref_dir):
  # Tests that the bump road scenario from matlab is consistent with the python generated one
  matlab_sim = load_sim(ref_dir,"ramp","passive")
  scen_cfg = load_json(ref_dir, "scenarios.json")
  scen_dt = scen_cfg["sim"]["dt"]

  _, py_sim_zrdot = make_road("ramp",scen_cfg["scenarios"]["ramp"],scen_dt)

  assert len(py_sim_zrdot) == len(matlab_sim["zr_dot"])
  assert np.allclose(py_sim_zrdot, matlab_sim["zr_dot"], rtol = 1e-6, atol = 1e-9)

def test_bump_matlab_matching(ref_dir):
  # Tests that the bump road scenario from matlab is consistent with the python generated one
  matlab_sim = load_sim(ref_dir,"bump","passive")
  scen_cfg = load_json(ref_dir, "scenarios.json")
  scen_dt = scen_cfg["sim"]["dt"]

  _, py_sim_zrdot = make_road("bump",scen_cfg["scenarios"]["bump"],scen_dt)

  assert len(py_sim_zrdot) == len(matlab_sim["zr_dot"])
  assert np.allclose(py_sim_zrdot, matlab_sim["zr_dot"], rtol = 1e-6, atol = 1e-9)


