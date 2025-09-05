# Weight states and inputs of an MPC problem
from solver_model import DynamicsModel
from parameters import Parameters

class MPCBaseModule:

    def __init__(self, settings):
        self.type = "objective"
        self._weights = []
        self._cost_functions = []
        self._variables_per_function = []
        self._weights_per_function = []


    # Add a variable that is weighted in the cost
    def weigh_variable(self, var_name, weight_names, cost_function):
        if type(weight_names) != list:
            weight_names = [weight_names]

        for weight_name in weight_names:
            self._weights.append(weight_name)
            
        self._weights_per_function.append(weight_names)
        self._variables_per_function.append(var_name)
        self._cost_functions.append(cost_function)

    def define_parameters(self, params):
        for param in self._weights:
            params.add(param)
        return params
    
    def get_value(self, model : DynamicsModel, params : Parameters, settings, stage_idx):
        cost = 0.0
        for idx, cost_function in enumerate(self._cost_functions):
            weights = []
            for cost_weight in self._weights_per_function[idx]:  # Retrieve the weight parameters for this cost function!
                weights.append(params.get(cost_weight))

            variable = model.get(self._variables_per_function[idx])  # Retrieve the state / input to be weighted

            # Add to the cost
            cost += cost_function(variable, weights)

        return cost