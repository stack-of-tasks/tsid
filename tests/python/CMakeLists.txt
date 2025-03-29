set(${PYWRAP}_TESTS
    # Constraint
    # ContactPoint
    # Contact
    Formulation
    Gravity
    RobotWrapper
    Solvers
    Tasks
    Trajectories
    Deprecations
    Measured6DWrench)

foreach(test ${${PYWRAP}_TESTS})
  add_python_unit_test("py-${test}" "tests/python/test_${test}.py"
                       "bindings/python")
endforeach(test ${${PYWRAP}_TESTS})
