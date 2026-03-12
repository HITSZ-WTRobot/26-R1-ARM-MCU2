add_library(VelocityProfileSCurve STATIC
        "src/s_curve.cpp"
)

target_include_directories(VelocityProfileSCurve
        PUBLIC
        src
)

# link dependencies if any
target_link_libraries(VelocityProfileSCurve PUBLIC VelocityProfile::Core)

# alias for external use
add_library(VelocityProfile::SCurve ALIAS VelocityProfileSCurve)
