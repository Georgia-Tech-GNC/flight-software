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

<<<<<<< HEAD
#ifndef __IDLE_H__
    #define __IDLE_H__
=======
>>>>>>> 70d328d11b7681ae405b01e8fe40bd24ff6d0163

void run_idle() {
        // Check for arming signal
        
        //Wait for signal to start sensor calibration.
        char signal_received[] = "NO";
        while (1){
            signal_received = ;//TODO: HAL and receiving through UART

            if (signal_received == "GO"){
            break;
            }

            //TODO: Vane Alignment Calibration upon receipt of another specified Xbee signal
        }

    if (signal_received) {
        STATEMACHINE = GROUND;
        first_iter = 1;
    }

}


#end