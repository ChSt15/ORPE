add_executable(router
    "${CMAKE_CURRENT_LIST_DIR}/router.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/uart_router.cpp"
)
set_target_properties(router PROPERTIES SUFFIX ".elf")
target_link_libraries(router PUBLIC rodos)


set(output_name sender)
add_executable(${output_name}
    "${CMAKE_CURRENT_LIST_DIR}/demo_topics.h"
    "${CMAKE_CURRENT_LIST_DIR}/demo_topics.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/gateway1.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/sender.cpp"
)
set_target_properties(${output_name} PROPERTIES SUFFIX ".elf")
target_link_libraries(${output_name} PUBLIC rodos)
add_custom_command(TARGET ${output_name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${output_name}.elf ${output_name}.bin
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)


set(output_name receiver1)
add_executable(${output_name}
    "${CMAKE_CURRENT_LIST_DIR}/demo_topics.h"
    "${CMAKE_CURRENT_LIST_DIR}/demo_topics.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/gateway1.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/receiver1.cpp"
)
set_target_properties(${output_name} PROPERTIES SUFFIX ".elf")
target_link_libraries(${output_name} PUBLIC rodos)
add_custom_command(TARGET ${output_name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${output_name}.elf ${output_name}.bin
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)


set(output_name receiver2)
add_executable(${output_name}
    "${CMAKE_CURRENT_LIST_DIR}/demo_topics.h"
    "${CMAKE_CURRENT_LIST_DIR}/demo_topics.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/gateway1.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/receiver2.cpp"
)
set_target_properties(${output_name} PROPERTIES SUFFIX ".elf")
target_link_libraries(${output_name} PUBLIC rodos)
add_custom_command(TARGET ${output_name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${output_name}.elf ${output_name}.bin
    WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
