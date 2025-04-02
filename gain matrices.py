import os
import numpy as np
import dartpy as dart

def compute_damping_stiffness(hrp4, J_k, alpha=0.1, beta=50.0):
    """
    Compute task-space damping (D_k) and stiffness (K_k) matrices using:
    D_k = alpha * M_k
    K_k = beta * M_k
    
    :param hrp4: The humanoid robot model.
    :param J_k: Task Jacobian matrix (n_k x 31).
    :param alpha: Scaling factor for damping.
    :param beta: Scaling factor for stiffness.
    :return: D_k (n_k x n_k), K_k (n_k x n_k)
    """
    # Get the full-body inertia matrix (31x31)
    M = hrp4.getMassMatrix()

    # Compute task-space inertia matrix M_k = (J_k * M^{-1} * J_k.T)^{-1}
    M_inv = np.linalg.inv(M)
    M_k = np.linalg.inv(J_k @ M_inv @ J_k.T)  # (n_k x n_k)

    # Compute D_k and K_k
    D_k = alpha * M_k  # n_k x n_k
    K_k = beta * M_k   # n_k x n_k

    return D_k, K_k

if __name__ == "__main__":
    # Create world and load HRP4 model
    world = dart.simulation.World()
    urdfParser = dart.utils.DartLoader()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    hrp4 = urdfParser.parseSkeleton(os.path.join(current_dir, "urdf", "hrp4.urdf"))
    world.addSkeleton(hrp4)

    # Get the Jacobian for a specific task (e.g., Center of Mass)
    J_com = hrp4.getCOMLinearJacobian(dart.dynamics.Frame.World())

    # Compute D_k and K_k for the CoM task
    D_com, K_com = compute_damping_stiffness(hrp4, J_com, alpha=0.1, beta=50.0)

    # Print results
    print("Matrice di smorzamento D_k (CoM):\n", D_com)
    print("Matrice di rigidità K_k (CoM):\n", K_com)
#if you want to compute gain matrices for feet or torso change J_com to J_lfoot = hrp4.getBodyNode('l_sole').getSpatialJacobian(dart.dynamics.Frame.World())
