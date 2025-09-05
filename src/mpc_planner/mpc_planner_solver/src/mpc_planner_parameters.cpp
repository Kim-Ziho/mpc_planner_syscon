#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_solver/acados_solver_interface.h>

namespace MPCPlanner{

void setSolverParameterAcceleration(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 0] = value;
}
void setSolverParameterAngularVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 1] = value;
}
void setSolverParameterVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 2] = value;
}
void setSolverParameterReferenceVelocity(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 3] = value;
}
void setSolverParameterContour(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 4] = value;
}
void setSolverParameterLag(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 5] = value;
}
void setSolverParameterTerminalAngle(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 6] = value;
}
void setSolverParameterTerminalContouring(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 7] = value;
}
void setSolverParameterSplineXA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 8] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 17] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 26] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 35] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 44] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 53] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 62] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 71] = value;
}
void setSolverParameterSplineXB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 9] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 18] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 27] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 36] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 45] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 54] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 63] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 72] = value;
}
void setSolverParameterSplineXC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 10] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 19] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 28] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 37] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 46] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 55] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 64] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 73] = value;
}
void setSolverParameterSplineXD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 11] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 20] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 29] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 38] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 47] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 56] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 65] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 74] = value;
}
void setSolverParameterSplineYA(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 12] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 21] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 30] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 39] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 48] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 57] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 66] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 75] = value;
}
void setSolverParameterSplineYB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 13] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 22] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 31] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 40] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 49] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 58] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 67] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 76] = value;
}
void setSolverParameterSplineYC(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 14] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 23] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 32] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 41] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 50] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 59] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 68] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 77] = value;
}
void setSolverParameterSplineYD(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 15] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 24] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 33] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 42] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 51] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 60] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 69] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 78] = value;
}
void setSolverParameterSplineStart(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 16] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 25] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 34] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 43] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 52] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 61] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 70] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 79] = value;
}
void setSolverParameterEgoDiscOffset(int k, AcadosParameters& params, const double value, int index){
	(void)index;
	params.all_parameters[k * 117 + 80] = value;
}
void setSolverParameterDecompA1(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 81] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 84] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 87] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 90] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 93] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 96] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 99] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 102] = value;
	else if(index == 8)
		params.all_parameters[k * 117 + 105] = value;
	else if(index == 9)
		params.all_parameters[k * 117 + 108] = value;
	else if(index == 10)
		params.all_parameters[k * 117 + 111] = value;
	else if(index == 11)
		params.all_parameters[k * 117 + 114] = value;
}
void setSolverParameterDecompA2(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 82] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 85] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 88] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 91] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 94] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 97] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 100] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 103] = value;
	else if(index == 8)
		params.all_parameters[k * 117 + 106] = value;
	else if(index == 9)
		params.all_parameters[k * 117 + 109] = value;
	else if(index == 10)
		params.all_parameters[k * 117 + 112] = value;
	else if(index == 11)
		params.all_parameters[k * 117 + 115] = value;
}
void setSolverParameterDecompB(int k, AcadosParameters& params, const double value, int index){
	if(index == 0)
		params.all_parameters[k * 117 + 83] = value;
	else if(index == 1)
		params.all_parameters[k * 117 + 86] = value;
	else if(index == 2)
		params.all_parameters[k * 117 + 89] = value;
	else if(index == 3)
		params.all_parameters[k * 117 + 92] = value;
	else if(index == 4)
		params.all_parameters[k * 117 + 95] = value;
	else if(index == 5)
		params.all_parameters[k * 117 + 98] = value;
	else if(index == 6)
		params.all_parameters[k * 117 + 101] = value;
	else if(index == 7)
		params.all_parameters[k * 117 + 104] = value;
	else if(index == 8)
		params.all_parameters[k * 117 + 107] = value;
	else if(index == 9)
		params.all_parameters[k * 117 + 110] = value;
	else if(index == 10)
		params.all_parameters[k * 117 + 113] = value;
	else if(index == 11)
		params.all_parameters[k * 117 + 116] = value;
}
}
