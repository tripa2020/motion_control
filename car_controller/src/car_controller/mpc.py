from __future__ import division
import numpy as np

from cs4750 import utils
from car_kinematics.kinematic_model import KinematicCarMotionModel
from car_controller.controller import BaseController


class ModelPredictiveController(BaseController):
    def __init__(self, **kwargs):
        self.__properties = {
            "car_length",
            "car_width",
            "collision_w",
            "error_w",
            "min_alpha",
            "max_alpha",
            "K",
            "T",
            "kinematics_params",
            "permissible_region",
            "map_info",
        }

        if not self.__properties.issubset(set(kwargs)):
            raise ValueError(
                "Invalid keyword argument provided",
                self.__properties.difference(set(kwargs)),
            )
        self.__dict__.update(kwargs)
        assert self.K > 0 and self.T > 0
        self.motion_model = KinematicCarMotionModel(
            self.car_length, **self.kinematics_params
        )

        super(ModelPredictiveController, self).__init__(
            **{k: kwargs[k] for k in set(kwargs) if k not in self.__properties}
        )

    def sample_controls(self):
        """Generate K sequences of (v=0 placeholder, alpha) of length T-1."""
        # Prepare control array: shape (K, T-1, 2)
        controls = np.zeros((self.K, self.T - 1, 2))

        # Evenly spaced steering angles
        alphas = np.linspace(self.min_alpha, self.max_alpha, self.K)
        # Fill the steering column (index 1) for all timesteps
        controls[:, :, 1] = alphas.reshape(self.K, 1)

        return controls

    def get_rollout(self, pose, controls, dt=0.1):
        """Vectorized rollout of K control sequences."""
        K, T = self.K, self.T
        assert controls.shape == (K, T - 1, 2)

        rollouts = np.empty((K, T, 3))
        rollouts[:, 0] = pose
        current = np.tile(pose, (K, 1))

        for t in range(T - 1):
            delta = self.motion_model.compute_changes(
                current, controls[:, t], dt
            )
            current = current + delta
            rollouts[:, t + 1] = current

        return rollouts

    def compute_distance_cost(self, rollouts, reference_xyt):
        """Distance cost = error_w * Euclidean distance at final state."""
        final_xy = rollouts[:, -1, :2]
        ref_xy   = reference_xyt[:2]
        dists    = np.linalg.norm(final_xy - ref_xy, axis=1)
        return self.error_w * dists

    def compute_collision_cost(self, rollouts, _):
        """Collision cost = collision_w * (# of collision timesteps)."""
        K, T = self.K, self.T
        flat = rollouts.reshape(K * T, 3)
        collisions = self.check_collisions_in_map(flat)
        collisions = collisions.reshape(K, T)
        counts = collisions.sum(axis=1)
        return self.collision_w * counts

    def compute_rollout_cost(self, rollouts, reference_xyt):
        d_cost = self.compute_distance_cost(rollouts, reference_xyt)
        c_cost = self.compute_collision_cost(rollouts, reference_xyt)
        return d_cost + c_cost

    def get_control(self, pose, reference_xytv, _):
        """
        1) Set v from reference,
        2) Roll out K sequences,
        3) Pick sequence with lowest cost,
        4) Return its first (v, alpha).
        """
        # Unpack reference: [x, y, theta, v_ref]
        assert reference_xytv.shape[0] == 4
        v_ref = reference_xytv[3]

        # Refresh control samples and apply v_ref
        self.sampled_controls = self.sample_controls()
        self.sampled_controls[:, :, 0] = v_ref

        # Generate rollouts and their costs
        rollouts = self.get_rollout(pose, self.sampled_controls, dt=0.1)
        costs    = self.compute_rollout_cost(rollouts, reference_xytv[:3])

        # Store for visualization
        with self.state_lock:
            self.rollouts = rollouts
            self.costs    = costs

        # Return the best first action
        best = np.argmin(costs)
        return self.sampled_controls[best, 0]

    def reset_params(self, **kwargs):
        # Allow updating only controller + BaseController params
        allowed = set(self._properties).union({
            "car_length", "car_width", "collision_w", "error_w",
            "min_alpha", "max_alpha", "K", "T",
            "kinematics_params", "permissible_region", "map_info"
        })
        bad = set(kwargs).difference(allowed)
        if bad:
            raise ValueError("Invalid keyword args:", bad)
        self.__dict__.update(kwargs)

    def reset_state(self):
        # Prepare fresh rollout/collision arrays
        self.sampled_controls = self.sample_controls()
        total = self.K * self.T
        self.map_poses  = np.zeros((total, 3))
        self.bbox_map   = np.zeros((total, 2, 4))
        self.collisions = np.zeros(total, dtype=bool)
        self.obstacle_map = ~self.permissible_region

        # Precompute car bounding box corners (in map units)
        half_l = self.car_length / 2.0 / self.map_info.resolution
        half_w = self.car_width  / 2.0 / self.map_info.resolution
        self.car_bbox = np.array([
            [ half_l,  half_w],
            [ half_l, -half_w],
            [-half_l,  half_w],
            [-half_l, -half_w],
        ])

    def check_collisions_in_map(self, poses):
        """Vectorized collision check against the obstacle map."""
        # Transform world → map grid
        utils.world_to_map(poses, self.map_info, out=self.map_poses)
        pts   = self.map_poses[:, :2]
        thetas= self.map_poses[:, 2]

        # Build rot matrices for each theta
        c, s = np.cos(thetas), np.sin(thetas)
        R = np.stack([ [c, -s], [s, c] ], axis=-1)  # shape (N, 2, 2)

        # Rotate bbox and translate to each pose
        corners = self.car_bbox[np.newaxis,...] @ R  # (N,4,2)
        corners = corners + pts[:, np.newaxis, :]

        # Round/truncate to grid indices
        idx = corners.astype(int)
        # Clip to map bounds
        idx[...,1] = np.clip(idx[...,1], 0, self.obstacle_map.shape[0]-1)
        idx[...,0] = np.clip(idx[...,0], 0, self.obstacle_map.shape[1]-1)

        # Check any corner in collision
        coll = self.obstacle_map[idx[...,1], idx[...,0]].max(axis=1)
        return coll