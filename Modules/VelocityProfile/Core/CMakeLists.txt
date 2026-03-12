add_library(__VelocityProfile_Core INTERFACE)

target_include_directories(__VelocityProfile_Core INTERFACE
        .
)

# link dependencies if any


# alias for external use
add_library(VelocityProfile::Core ALIAS __VelocityProfile_Core)
