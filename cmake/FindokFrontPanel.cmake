add_library(okFrontPanel::okFrontPanel SHARED IMPORTED)

target_include_directories(okFrontPanel::okFrontPanel INTERFACE
    ${PROJECT_SOURCE_DIR}/third_party/FrontPanel
)

if(APPLE)
    set_target_properties(okFrontPanel::okFrontPanel PROPERTIES
        IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/third_party/FrontPanel/libokFrontPanel.dylib
    )
elseif(WIN32)
    set_target_properties(okFrontPanel::okFrontPanel PROPERTIES
        IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/third_party/FrontPanel/okFrontPanel.dll
        IMPORTED_IMPLIB ${PROJECT_SOURCE_DIR}/third_party/FrontPanel/okFrontPanel.lib
    )
else()
    set_target_properties(okFrontPanel::okFrontPanel PROPERTIES
        IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/third_party/FrontPanel/libokFrontPanel.so
    )
endif()

set(okFrontPanel_LIBRARIES okFrontPanel::okFrontPanel)
set(okFrontPanel_FOUND TRUE)