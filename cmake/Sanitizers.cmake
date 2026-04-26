option(MOON_PLANNER_ENABLE_ASAN "Enable AddressSanitizer for debug builds" OFF)

function(moon_planner_apply_sanitizers target_name)
  if(MOON_PLANNER_ENABLE_ASAN)
    target_compile_options(${target_name} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target_name} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()
