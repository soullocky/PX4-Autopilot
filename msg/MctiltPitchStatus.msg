# MCTilt pitch source and filtered reference for flight-test diagnostics.
uint64 timestamp
float32 requested_pitch # radians; zero on external reference timeout
float32 applied_pitch # radians after filtering and slew limit
bool external_active
bool input_valid

