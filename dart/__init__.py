"""DART - Decision-Adaptive Reward Targeting.

Online target-selection + reactive navigation for a single robot handling
many tasks whose value decays over time (perishable reward).

Pure-Python core (no ROS2 in this package). A ROS2 node layer can be added
later on top of these modules. See DART_concept.md for the full concept.

Module map:
    scoring       -> Net_i computation (concept section 4)
    decider       -> argmax selection + hysteresis switching (section 5)
    navigator     -> reactive Bug2 drive + LiDAR avoid (steps 5-6)
    localization  -> pose source (AMCL wrapper later; stub for now)
    sim           -> headless sim comparing DART vs nearest-first
"""

__all__ = ["scoring", "decider", "navigator"]
__version__ = "0.0.1"
