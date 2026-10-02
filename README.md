# bot1

The computer is the brain. The robot is the body.

![The quad robot](docs/robot.jpg)

The photo shows the four-legged robot. The ESP32 sits in the middle, and the red light means the board is powered. The wires go to eight servos. The motors on the limbs only rotate the legs sideways. The motors on the feet raise and lower each foot.

The brain runs on the computer, not on the board. The neurons come from the [FlyWire Codex FAFB fruit-fly brain](https://codex.flywire.ai/?dataset=fafb). The loaded set is the heading and steering group: 167 neurons, including EPG, PEN, PEG, PFL, DNa01, and DNa02, with 1,968 synapses between them. The computer uses these neurons to decide the current heading and whether to turn or walk forward, then sends that command to the ESP32. The robot only moves the eight servos.

The full FlyWire brain has about 140,000 neurons. The computer is running this small group taken from the FAFB data, not the whole brain. Making it walk on its own comes later.

A related project is [Fly Brain Bridge](https://www.doriantodd.com/projects/fly-brain-bridge/) by Dorian Todd. It runs a much larger male-fly nervous system and drives another quadruped. It is a reference for this robot, not the circuit loaded here.
