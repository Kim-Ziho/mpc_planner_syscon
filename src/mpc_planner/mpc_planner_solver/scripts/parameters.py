import copy
import yaml
import casadi as cd  # Acados


class Parameters:
    def __init__(self):
        self._params = dict()
        self.parameter_bundles = dict()  # Used to generate function names in C++ with an integer parameter

        self._param_idx = 0
        self._p = None

    def add(self, parameter, bundle_name=None):
        if parameter not in self._params:
            self._params[parameter] = self._param_idx
        
        if bundle_name is None:
            bundle_name = parameter

        if bundle_name not in self.parameter_bundles.keys():
            self.parameter_bundles[bundle_name] = [copy.deepcopy(self._param_idx)]
        else:
            self.parameter_bundles[bundle_name].append(copy.deepcopy(self._param_idx))
        
        self._param_idx += 1



    def length(self):
        return self._param_idx
    
    def get(self, parameter):
        if self._p is None:
            print("Load parameters before requesting them!")
            return None
        return self._p[self._params[parameter]]
    
    def load(self, p):
        self._p = p
    
    def load_acados_parameters(self):

        self._p = []
        for param in self._params.keys():
            par = cd.SX.sym(param, 1)
            self._p.append(par)

        self.load(self._p)

    def get_acados_p(self):
        return self._p

    def print(self):
        print("Parameters:")
        for param, idx in self._params.items():
            print(f"  {idx}: {param}")

    def get_acados_parameters(self):
        result = None
        for param in self._params.keys():
            if result is None:
                result = self.get(param)
            else:
                result = cd.vertcat(result, self.get(param))

        return result

    def has_parameter(self, parameter):
        return parameter in self._params
    
    def save_map(self, file):
        map = self._params
        map["num parameters"] = self._param_idx
        with open(file, "w") as outfile:
            yaml.dump(map, outfile, default_flow_style=False)