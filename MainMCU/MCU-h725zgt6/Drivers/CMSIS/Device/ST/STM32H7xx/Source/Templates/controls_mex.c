#include "C:/Program Files/MATLAB/R2023b/extern/include/mex.h"
#include "controls.h"  // Include your LQR controller header

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

    controller ctrl;
    initialize_controls(&ctrl);
    

    // Assume input is the current state vector (array of floats)
    float current_state[9];
    float elapsed_time;

    // Read input from MATLAB (assumes prhs[0] is a double array)
    double *input_1 = mxGetPr(prhs[0]);
    for (int i = 0; i < 9; i++) {
        current_state[i] = (float)input_1[i];
    }

    double *input_2 = mxGetPr(prhs[1]);
    elapsed_time = (float)(*input_2);

    // Call the C-based LQR function (implement this in controls.c)
    run_controls(&ctrl, current_state, elapsed_time);

    // Prepare MATLAB output (control inputs as a double array)
    plhs[0] = mxCreateDoubleMatrix(1, 3, mxREAL);
    double *output = mxGetPr(plhs[0]);
    output[0] = ctrl.M_roll;
    output[1] = ctrl.M_pitch;
    output[2] = ctrl.M_yaw;
}
