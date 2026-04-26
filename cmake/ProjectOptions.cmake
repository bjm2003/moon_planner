function(moon_planner_apply_project_options target_name)
  target_compile_options(${target_name} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror=return-type
  )
endfunction()
