cmake_minimum_required(VERSION 3.16)
project(modbus_rtu_protocol VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(BUILD_EXAMPLES "Build example applications" ON)
option(BUILD_WITH_DDS "Build with FastDDS support (generates IDL types)" OFF)
option(ENABLE_LOGGING "Enable debug logging" OFF)

# Logging flag
if(ENABLE_LOGGING)
    add_compile_definitions(ENABLE_MODBUS_RTU_LOGGING)
endif()

# Header-only library target
add_library(modbus_rtu INTERFACE)
target_include_directories(modbus_rtu INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Required system libraries
find_package(Threads REQUIRED)

# Examples
if(BUILD_EXAMPLES)
    add_executable(modbus_rtu_example examples/modbus_rtu_example.cpp)
    target_link_libraries(modbus_rtu_example PRIVATE modbus_rtu Threads::Threads)
    target_compile_definitions(modbus_rtu_example PRIVATE ENABLE_MODBUS_RTU_LOGGING)
endif()

# FastDDS IDL generation (optional)
if(BUILD_WITH_DDS)
    find_package(fastcdr 2 REQUIRED)
    find_package(fastdds 3 REQUIRED)

    # Check for fastddsgen
    find_program(FASTDDSGEN fastddsgen)
    if(FASTDDSGEN)
        message(STATUS "Found fastddsgen: ${FASTDDSGEN}")

        # Generate C++ types from IDL
        set(IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/idl/ModbusRTU.idl)
        set(GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated)

        add_custom_command(
            OUTPUT
                ${GENERATED_DIR}/ModbusRTU.hpp
                ${GENERATED_DIR}/ModbusRTUCdrAux.hpp
                ${GENERATED_DIR}/ModbusRTUPubSubTypes.hpp
                ${GENERATED_DIR}/ModbusRTUPubSubTypes.cxx
                ${GENERATED_DIR}/ModbusRTUTypeObjectSupport.hpp
                ${GENERATED_DIR}/ModbusRTUTypeObjectSupport.cxx
            COMMAND ${FASTDDSGEN} -d ${GENERATED_DIR} -typeros2 ${IDL_FILE}
            DEPENDS ${IDL_FILE}
            COMMENT "Generating C++ types from ModbusRTU.idl"
        )

        add_library(modbus_rtu_types
            ${GENERATED_DIR}/ModbusRTUPubSubTypes.cxx
            ${GENERATED_DIR}/ModbusRTUTypeObjectSupport.cxx
        )
        target_include_directories(modbus_rtu_types PUBLIC ${GENERATED_DIR})
        target_link_libraries(modbus_rtu_types PUBLIC fastcdr fastdds)
    else()
        message(WARNING "fastddsgen not found. Skipping IDL generation.")
    endif()
endif()

# Install rules
install(FILES include/modbus_rtu.h DESTINATION include)
install(FILES idl/ModbusRTU.idl DESTINATION share/idl)
install(DIRECTORY schemas/ DESTINATION share/schemas)
install(DIRECTORY descriptors/ DESTINATION share/descriptors)

if(BUILD_EXAMPLES)
    install(TARGETS modbus_rtu_example DESTINATION bin)
endif()

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/modbus_rtu_protocol-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Summary
message(STATUS "")
message(STATUS "ModbusRTU Protocol Configuration:")
message(STATUS "  Version:       ${PROJECT_VERSION}")
message(STATUS "  Build examples: ${BUILD_EXAMPLES}")
message(STATUS "  Build with DDS: ${BUILD_WITH_DDS}")
message(STATUS "  Enable logging: ${ENABLE_LOGGING}")
message(STATUS "")
