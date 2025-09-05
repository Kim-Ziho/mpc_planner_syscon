import casadi as cd
import numpy as np
import yaml

def numpy_to_casadi(x: np.array) -> cd.SX:
        result = None
        for param in x:
            if result is None:
                result = param
            else:
                result = cd.vertcat(result, param)
        return result


class DynamicsModel:

    def __init__(self):
        self.nu = 0  # number of control variables
        self.nx = 0  # number of states

        self.states = []
        self.inputs = []

        self.lower_bound = []
        self.upper_bound = []

    def continuous_model(self, x, u):
        raise NotImplementedError("Implement continuous model in subclass")
    
    def get_xinit(self):
        return range(self.nu, self.get_nvar() - 1)
    
    def get(self, state_or_input):
        if state_or_input in self.states:
            i = self.states.index(state_or_input)
            return self._z[self.nu + i]
        elif state_or_input in self.inputs:
            i = self.inputs.index(state_or_input)
            return self._z[i]
        else:
            raise IOError(f"Requested a state or input `{state_or_input}' that was neither a state nor an input for the selected model")

    def acados_symbolics(self):
        x = cd.SX.sym("x", self.nx)  # [px, py, vx, vy]
        u = cd.SX.sym("u", self.nu)  # [ax, ay]
        z = cd.vertcat(u, x)
        self.load(z)
        return z
    
    def get_acados_dynamics(self):
        self._x_dot = cd.SX.sym("x_dot", self.nx)

        f_expl = numpy_to_casadi(self.continuous_model(self._z[self.nu :], self._z[: self.nu]))
        f_impl = self._x_dot - f_expl
        return f_expl, f_impl
    
    def get_x(self):
        return self._z[self.nu :]

    def get_u(self):
        return self._z[: self.nu]
    
    def get_x_dot(self):
        return self._x_dot

    def load(self, z):
        self._z = z

    def get_nvar(self):
        return self.nu + self.nx
    
    def save_map(self, file):
        map = dict()
        for idx, state in enumerate(self.states):
            map[state] = ["x", idx + self.nu, self.get_bounds(state)[0], self.get_bounds(state)[1]]

        for idx, input in enumerate(self.inputs):
            map[input] = ["u", idx, self.get_bounds(input)[0], self.get_bounds(input)[1]]

        with open(file, "w") as outfile:
            yaml.dump(map, outfile, default_flow_style=False)

    def get_bounds(self, state_or_input):
        if state_or_input in self.states:
            i = self.states.index(state_or_input)
            return (
                self.lower_bound[self.nu + i],
                self.upper_bound[self.nu + i],
                self.upper_bound[self.nu + i] - self.lower_bound[self.nu + i],
            )
        elif state_or_input in self.inputs:
            i = self.inputs.index(state_or_input)
            return (
                self.lower_bound[i],
                self.upper_bound[i],
                self.upper_bound[i] - self.lower_bound[i],
            )
        else:
            raise IOError(f"Requested a state or input `{state_or_input}' that was neither a state nor an input for the selected model")


class ContouringSecondOrderUnicycleModel(DynamicsModel):

    def __init__(self):
        super().__init__()
        self.nu = 2  # number of control variables
        self.nx = 6  # number of states

        self.states = ["x", "y", "psi", "v", "spline", "slack"]
        self.inputs = ["a", "w"]

        self.lower_bound = [-2.0, -0.8, -2000.0, -2000.0, -np.pi * 4, -0.01, -1.0, 0.0]  # v -0.01
        self.upper_bound = [2.0, 0.8, 2000.0, 2000.0, np.pi * 4, 3.0, 10000.0, 5000.0]  # w 0.8

    def continuous_model(self, x, u):

        a = u[0]
        w = u[1]
        psi = x[2]
        v = x[3]

        return np.array([v * cd.cos(psi), v * cd.sin(psi), w, a, v, 0.0])

    