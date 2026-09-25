*** Settings ***
Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown
Resource            ${RENODEKEYWORDS}
Library             ConfigLoader.py

*** Test Cases ***
Verify Three Unique Tasks Printed
    Load Config    ${VARIABLE_FILE}
    # 1. Load your platform script
    Execute Command          path add @${PROJECT_ROOT}
    Execute Command          include @${INIT_RESC_FILE}

    Execute Command          sysbus LoadELF @${FIRMWARE_ELF}
    
    Create Terminal Tester    ${DEBUG_UART}

    Start Emulation

    Wait For Line On Uart    Blink 1
    Wait For Line On Uart    Blink 2
    Wait For Line On Uart    Blink 3
