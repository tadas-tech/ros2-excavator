# State of one hydraulic cylinder, measured between its two mounting pins.

string name

float64 length      # pin-to-pin length now (m)
float64 min_length  # shortest pin-to-pin length over the joint's range (m): fully retracted
float64 max_length  # longest pin-to-pin length over the joint's range (m): fully extended

float64 stroke      # extension from fully retracted: length - min_length (m)
float64 velocity    # rate of change of length (m/s), positive when extending

# min_length, max_length and stroke are NaN when the cylinder's base and tip
# links are not joined by a single revolute joint, so the range is unknown.
