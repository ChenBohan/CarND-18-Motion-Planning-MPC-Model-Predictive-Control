# CarND Controls Quizzes

Quizzes for *Vehicle Models* and *Model Predictive Control* sections.

1. [Global Kinematic Model Quiz](./global_kinematic_model) - Implement the *Global Kinematic Model*.
2. [Polynomial Fitting Quiz](./polyfit) - Fit and evaluate polynomials.
3. [Mind The Line Quiz](./mpc_to_line) - Implement MPC and minimize cross track and orientation errors to a straight line trajectory.  See [this document](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md) for setup tips for executing the plotting code in the ```MPC.cpp``` solution file.

To do a quiz:

1. Go to quiz directory.
2. Make a build directory with `mkdir build`.
3. Change into the build directory, `cd build`.
4. Compile the project, `cmake .. && make`.

A solution for each quiz is presented in the solution directory.

## Dependencies

The *Global Kinematic Quiz* and *Polynomial Fitting* quizzes have all the dependencies in repo. For the *MPC* quiz
you'll have to install Ipopt and CppAD.  Please refer to [this document](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md) for installation instructions.


# CarND-18-Motion-Planning-MPC-Model-Predictive-Control

### Global Kinematic Model

```cpp
next_state(0) = x + v * cos(psi) * dt;
next_state(1) = y + v * sin(psi) * dt;
next_state(2) = psi + v / Lf * delta * dt;
next_state(3) = v + a * dt;
```

### Cross Track Error

CTE is the difference between the line and the current vehicle position.
```cpp
double cte = polyeval(coeffs, x) - y;
```

### Orientation Error

```cpp
// derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
double epsi = psi - atan(coeffs[1]);
```

### MPC

Solve the model given an initial state and return the next state and actuations as a vector.

1. Set the initial variable values
```cpp
vars[x_start] = x;
vars[y_start] = y;
vars[psi_start] = psi;
vars[v_start] = v;
vars[cte_start] = cte;
vars[epsi_start] = epsi;
```

2. Set lower and upper limits.
```cpp
// The upper and lower limits of delta are set to -25 and 25
// degrees (values in radians).
for (int i = delta_start; i < a_start; ++i) {
  vars_lowerbound[i] = -0.436332;
  vars_upperbound[i] = 0.436332;
}

// Acceleration/decceleration upper and lower limits.
// NOTE: Feel free to change this to something else.
for (int i = a_start; i < n_vars; ++i) {
  vars_lowerbound[i] = -1.0;
  vars_upperbound[i] = 1.0;
}
```

3. Set lower and upper limits for constraints.
```cpp
constraints_lowerbound[x_start] = x;
constraints_lowerbound[y_start] = y;
constraints_lowerbound[psi_start] = psi;
constraints_lowerbound[v_start] = v;
constraints_lowerbound[cte_start] = cte;
constraints_lowerbound[epsi_start] = epsi;
```

4. Solve the problem.
```cpp
CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
```
      
5. Check some of the solution values.
```cpp
ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
```



```
