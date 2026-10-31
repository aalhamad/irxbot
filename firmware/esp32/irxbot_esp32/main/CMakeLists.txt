idf_component_register(SRCS "irxbot_esp32.c"
                            "task_uart.c"
                            "task_lighting.c"
                            "task_sensors.c"
                            "task_telemetry.c"
                            "task_alarm.c"
                            "task_display.c"
                    INCLUDE_DIRS ".")
