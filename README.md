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

### Overview

We use MPC to follow the trajectory along a line.

Steps:

1. Set N and dt.

2. Fit the polynomial to the waypoints.

3. Calculate initial cross track error and orientation error values.

4. Define the components of the cost function (state, actuators, etc).

5. Define the model constraints. 

### Global Kinematic Model

```cpp
next_state(0) = x + v * cos(psi) * dt;
next_state(1) = y + v * sin(psi) * dt;
next_state(2) = psi + v / Lf * delta * dt;
next_state(3) = v + a * dt;
```

### Tools

#### Ipopt

Ipopt is the tool we'll be using to optimize the control inputs. It's able to find locally optimal values (non-linear problem!) while keeping the constraints set directly to the actuators and the constraints defined by the vehicle model.

#### CppAD

CppAD is a library we'll use for automatic differentiation.

### Main

#### Fitting a polynomial to the waypoints

```python
auto coeffs = polyfit(ptsx, ptsy, 1);
```

#### Calculating the cross track and orientation error

```cpp
double cte = polyeval(coeffs, x) - y;
```

```cpp
// derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
double epsi = psi - atan(coeffs[1]);
```

### MPC

Solve the model given an initial state and return the next state and actuations as a vector.

#### 1. Set the initial variable values
```cpp
vars[x_start] = x;
vars[y_start] = y;
vars[psi_start] = psi;
vars[v_start] = v;
vars[cte_start] = cte;
vars[epsi_start] = epsi;
```

#### 2. Set lower and upper limits.
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

#### 3. Set lower and upper limits for constraints.
```cpp
// Lower and upper limits for constraints
// All of these should be 0 except the initial
// state indices.
Dvector constraints_lowerbound(n_constraints);
Dvector constraints_upperbound(n_constraints);
for (int i = 0; i < n_constraints; ++i) {
  constraints_lowerbound[i] = 0;
  constraints_upperbound[i] = 0;
}
constraints_lowerbound[x_start] = x;
constraints_lowerbound[y_start] = y;
constraints_lowerbound[psi_start] = psi;
constraints_lowerbound[v_start] = v;
constraints_lowerbound[cte_start] = cte;
constraints_lowerbound[epsi_start] = epsi;

constraints_upperbound[x_start] = x;
...
```

#### 4. Solve the problem.
```cpp
CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
```
      
#### 5. Check some of the solution values.
```cpp
ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
```

### FG_eval

#### Constructor

- `coeffs` are the coefficients of the fitted polynomial. `coeffs` will be used by the cross track error and heading error equations.

#### Input

- `fg` is a vector containing the cost function and vehicle model/constraints
- `vars` is a vector containing the variable values (state & actuators) used by the cost function and model.

#### Cost function

1. Based on the reference state.

cross-track error, heading error, and velocity error.

```python
fg[0] += CppAD::pow(vars[cte_start + t], 2);
fg[0] += CppAD::pow(vars[epsi_start + t], 2);
fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
```

2. Minimize the use of actuators.

A further enhancement is to constrain erratic control inputs.

```python
fg[0] += CppAD::pow(vars[delta_start + t], 2);
fg[0] += CppAD::pow(vars[a_start + t], 2);
```

3. Minimize the value gap between sequential actuations.

To make control decisions more consistent, or smoother.

```python
fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
```

#### Setup Constraints

1. Initial constraints

We initialize the model to the initial state.

```python
fg[1 + x_start] = vars[x_start];
fg[1 + y_start] = vars[y_start];
fg[1 + psi_start] = vars[psi_start];
fg[1 + v_start] = vars[v_start];
fg[1 + cte_start] = vars[cte_start];
fg[1 + epsi_start] = vars[epsi_start];
```

2. The rest of the constraints

All the other constraints based on the vehicle model.

Previously, we have set the corresponding `constraints_lowerbound` and the `constraints_upperbound` values to 0. That means the solver will force this value of `fg` to always be 0.

For example:

```python
for (int t = 1; t < N ; ++t) {
  // psi, v, delta at time t
  AD<double> psi0 = vars[psi_start + t - 1];
  AD<double> v0 = vars[v_start + t - 1];
  AD<double> delta0 = vars[delta_start + t - 1];

  // psi at time t+1
  AD<double> psi1 = vars[psi_start + t];

  // how psi changes
  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
}
```

Coding up the other parts of the model is similar:

```python
// The idea here is to constraint this value to be 0.
//
// Recall the equations for the model:
// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
// v_[t+1] = v[t] + a[t] * dt
// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] =
    cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] =
    epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
