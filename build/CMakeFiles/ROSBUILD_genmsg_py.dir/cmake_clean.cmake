FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/pf_drone_state_estimation/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/pf_drone_state_estimation/msg/__init__.py"
  "../src/pf_drone_state_estimation/msg/_Measurement_data.py"
  "../src/pf_drone_state_estimation/msg/_Feature_Keypoint.py"
  "../src/pf_drone_state_estimation/msg/_Feature_msg.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
