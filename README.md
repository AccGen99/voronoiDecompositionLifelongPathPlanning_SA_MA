# minEnergy_Cornering_LifelongPathPlanning_SA_MA
M.Tech Thesis Project @ Systems, Dynamics &amp; Control Lab, IITKGP

Implementation of paper - Minimum-energy Cornering Trajectory Planning with Self-rotation for Three-wheeled Omni-directional Mobile Robots

Lifelong path planning means the robot(s) will keep receiving the target(s) to reach and, based on remaining budget(s) and estimated cost of travel, will decide whether to accept the target or ask for a different one.

This is multi-agent branch where target assignment will depend on voronoi decomposition of the area centred on the agent and cost function as the energy required by agent to reach that point. The agent target within voronoi region will accept that one. In case any unmatched targets/agents remain, they will accept the next best target, outside region and less energy among all remaining ones
