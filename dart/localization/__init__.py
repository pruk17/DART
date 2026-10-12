"""Localization sub-package.

Provides the robot pose to the rest of DART. In the sim this is a ground-truth
pose; on TurtleBot it will wrap AMCL (LiDAR scan-matching against a SLAM map
built once at setup). The map is used ONLY for localization and goal
coordinates - NOT for global path planning.
"""
