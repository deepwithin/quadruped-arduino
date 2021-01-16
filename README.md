# Quadruped - Arduino codes

- A simple approach to make a central pattern generator for quadruped was tried. The locus of end point of leg for swing phase can be treated as semi-ellipse and the stance phase as straight line. 

- Simple planar inverse kinematics for 2 servo per leg was written.

- For developing a gait simple sinusoidal cycle was used and phase differences in multiples of pi wrt one of the leg.

- Coordinates are generated and fed to inverse kinematics function which returns servo angles.


Our aim was to test the algo on a smaller protoype. This repo has all the trial codes. Later a similar approach is found to be used in Eric the robo dog post on osmbed website though his build was much cooler and costlier than ours.