import os
import sys
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(HERE, "..", "..", "mpc_planner_solver", "scripts"))

from mpc_base import MPCBaseModule
from contouring import ContouringModule
from decomp_constraints import DecompConstraintModule
from solver_model import ContouringSecondOrderUnicycleModel
from generate_solver import generate_solver

def load_settings():
    path = os.path.join(HERE, "..", "config", "settings.yaml")
    with open(path, "r") as stream:
        settings = yaml.safe_load(stream)
    print("Settings :", path)
    return settings


    
settings = load_settings()

model = ContouringSecondOrderUnicycleModel()

modules = [
    MPCBaseModule(settings),
    ContouringModule(settings),
    DecompConstraintModule(settings)
]

# MPCBaseModule
# input [a, w] penalty
modules[0].weigh_variable(var_name='a' ,weight_names="acceleration", 
                          cost_function=lambda x, w: w[0] * x**2)
modules[0].weigh_variable(var_name='w' ,weight_names="angular_velocity", 
                          cost_function=lambda x, w: w[0] * x**2)
# velocity following error
modules[0].weigh_variable(var_name='v' ,weight_names=["velocity", "reference_velocity"], 
                          cost_function=lambda x, w: w[0] * (x-w[1])**2)

generate_solver(modules, model, settings)