# Output of radar data processing
#
uint32 total_targets
# Filtered range spectrum
float32[256] mti_out
# Up to 10 targets
float32[10] target_strength
float32[10] target_position
float32[10] target_angle
# Doppler spectrum of each target
float32[1280] doppler_spectrum
# Test string
string test
