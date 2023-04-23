set(controller_SRC
  TrackDesiredForce.cpp
)

set(controller_HDR
  TrackDesiredForce.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DTrackDesiredForce_EXPORTS")

