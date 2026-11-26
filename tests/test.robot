*** Settings ***
Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown
Resource            ${RENODEKEYWORDS}
Library             ConfigLoader.py

*** Test Cases ***
Verify Initialization
    Load Config    ${PLATFORM_YAML}
    Execute Command    path add @${PROJECT_ROOT}
    Execute Command    include @${INIT_RESC_FILE}

    Execute Command    sysbus LoadELF @${FIRMWARE_ELF}
    
    Create Terminal Tester    sysbus.${uart[0]['renode_uart_id']}
    Start Emulation

    Wait For Line On Uart    Initializing...
    Wait For Line On Uart    Done initializing.
