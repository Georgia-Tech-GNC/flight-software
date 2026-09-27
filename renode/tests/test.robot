*** Settings ***
Suite Setup         Setup
Suite Teardown      Teardown
Test Setup          Reset Emulation
Test Teardown       Test Teardown
Resource            ${RENODEKEYWORDS}
Library             ConfigLoader.py

*** Test Cases ***
Verify Three Unique Tasks Printed
    Load Config    ${PLATFORM_YAML}
    Execute Command    path add @${PROJECT_ROOT}
    Execute Command    include @${INIT_RESC_FILE}

    Execute Command    sysbus LoadELF @${FIRMWARE_ELF}
    
    Create Terminal Tester    ${debug_uart.renode_uart_id}
    ${tester_led1}=    Create LED Tester    sysbus.led_1
    ${tester_led2}=    Create LED Tester    sysbus.led_2
    ${tester_led3}=    Create LED Tester    sysbus.led_3

    Start Emulation

    Wait For Line On Uart    Blink 1
    Assert LED State         true     timeout=1    testerId=${tester_led1}
    Assert LED State         false    timeout=1    testerId=${tester_led1}

    Wait For Line On Uart    Blink 2
    Assert LED State         true     timeout=1    testerId=${tester_led2}
    Assert LED State         false    timeout=1    testerId=${tester_led2}

    Wait For Line On Uart    Blink 3
    Assert LED State         true     timeout=1    testerId=${tester_led3}
    Assert LED State         false    timeout=1    testerId=${tester_led3}
