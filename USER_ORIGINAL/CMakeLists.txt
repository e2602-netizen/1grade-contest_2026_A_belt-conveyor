add_library(USER_ORIGINAL_Lib STATIC)

target_sources(USER_ORIGINAL_Lib PRIVATE 
    Src/UserOriginal.cpp
    # write your source files here
)

target_include_directories(USER_ORIGINAL_Lib PUBLIC 
    Inc
)

target_link_libraries(USER_ORIGINAL_Lib PUBLIC 
    stm32cubemx
    ARRC_STM32HAL_Lib
)