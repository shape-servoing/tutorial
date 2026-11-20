"""
Data-driven Jacobian-based shape servoing: the tutorial, as code.
=================================================================

    rod              the simulated object (Kirchhoff rod, quasistatic equilibrium)
    running_example  robot coordinates, grasps, and the Plant q -> s
    features         feature maps: point coordinates, edge vectors, turning angles
    jacobian         exact Jacobian, offline regression, Broyden update
    control          control law and the shape-servoing loop (Algorithm 1)
    analysis         SVD, subspaces, and the local stability condition
    failures         controllability, observability, and mechanical instability
    spatial          compact 3-D rod, surface ARAP, and volumetric ARAP models
"""
from .analysis import (condition_number, damped_inverse, descent_angles, descent_margin,
                       lyapunov_rate, manipulability, null_space, null_space_projector,
                       numerical_rank, range_split, scaled, singular_triplets)
from .control import FeatureTask, Iteration, ShapeServoingLoop, control_law
from .features import (FEATURE_MAPS, feature_dimension, feature_map_jacobian, feature_vector,
                       rigid_motion)
from .jacobian import broyden_update, numerical_jacobian, offline_estimate
from .rod import Rod
from .spatial import SpatialModel, spatial_spec
from .running_example import (DOF_NAMES, N_POINTS, PROBE_AMPLITUDES, PROBE_FRACTIONS,
                              Q_ARCH, Q_NOMINAL, Q_REACHABLE, Q_RING, Q_STRAIGHT,
                              Q_STRAIGHT_MOVED, Plant, probe_motions, rigid_grasp,
                              ring_plant, target_from_grasp, unfeasible_points,
                              unreachable_points)
