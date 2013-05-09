FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/pf_drone_state_estimation/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/pf_drone_state_estimation/Measurement_data.h"
  "../msg_gen/cpp/include/pf_drone_state_estimation/Feature_Keypoint.h"
  "../msg_gen/cpp/include/pf_drone_state_estimation/Feature_msg.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
