gcc -I .\Common\Inc\ -I .\Drivers\CMSIS\DSP\Include\ -I .\Drivers\CMSIS\Include\ -c .\Common\Src\controls.c
gcc -I .\Common\Inc\ -I .\Drivers\CMSIS\DSP\Include\ -I .\Drivers\CMSIS\Include\ -c .\Common\Src\lqr_gains_data.c
gcc -I .\Common\Inc\ -I .\Drivers\CMSIS\DSP\Include\ -I .\Drivers\CMSIS\Include\ -c .\Common\Src\reference_state_data.c
gcc -I .\Common\Inc\ -I .\Drivers\CMSIS\DSP\Include\ -I .\Drivers\CMSIS\Include\ -c .\Common\Src\controls_unittests.c
gcc -I .\Common\Inc\ -I .\Drivers\CMSIS\DSP\Include\ -I .\Drivers\CMSIS\Include\ -c .\Common\Src\controls_mex.c
gcc .\controls.o .\controls_unittests.o .\reference_state_data.o .\lqr_gains_data.o -o executable
.\executable