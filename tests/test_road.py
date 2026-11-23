from quarter_car.scenarios import time_vector
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