/**
 * @file Idle.c
 * @author Albert Zheng
 * @brief Header file for idle state
 * 
 * Copyright 2024 Georgia Tech. All rights reserved.
 * Copyrighted materials may not be further disseminated.
 * This file must not be made publicly available anywhere.
*/

#include "../../Inc/States/Idle.h"

#ifndef __GROUND_H__
    #define __GROUND_H__

void run_idle() {
        // Check for arming signal
        
        //Wait for signal to start sensor calibration.
        char signal_received[] = "NO";
        while (1){
            signal_received = ;//TODO: HAL and receiving through UART

            if (signal_received == "GO"){
            break;
            }
        }

        //TODO: Initialize sensors
        //TODO: Write an EKF to do the sensor calibration.

        //Ground calibration is now complete. Send Xbee signal to ground station that calibration is complete and rocket is ready to be launched.
        char message_for_launch_readiness[] = "GOFORLAUNCH";
        //TODO: Line of code that transmits through HAL UART to controls MCU that vehicle is launch ready.

    if (signal_received) {
        STATEMACHINE = GROUND;
    }

}


#endif