SET(${PYWRAP}_TESTS
  #Constraint
  #ContactPoint
  #Contact
  Formulation
  Gravity
  RobotWrapper
  #Solvers
  Tasks
  Trajectories
  )

FOREACH(test ${${PYWRAP}_TESTS})
  ADD_PYTHON_UNIT_TEST("py-${test}" "tests/python/test_${test}.py" "bindings/python")
ENDFOREACH(test ${${PYWRAP}_TESTS})
