import numpy as np
import os
import pybullet as p
import random
from cliport.tasks import primitives
from cliport.tasks.grippers import Spatula
from cliport.tasks.task import Task
from cliport.utils import utils
import numpy as np
from cliport.tasks.task import Task
from cliport.utils import utils

class CylinderLineAlignment(Task):
    """Pick up a blue cylinder and align it along a green line marked on the tabletop without crossing the line's boundaries."""

    def __init__(self):
        super().__init__()
        self.max_steps = 10
        self.lang_template = "align the blue cylinder along the green line"
        self.task_completed_desc = "done aligning."
        self.additional_reset()

    def reset(self, env):
        super().reset(env)

        # Add line.
        line_size = (0.12, 0.02, 0.005)  # x, y, z dimensions for the asset size
        line_pose = self.get_random_pose(env, line_size)
        line_urdf = self.fill_template('line/single-green-line-template.urdf', {'DIM': line_size})
        env.add_object(line_urdf, line_pose, 'fixed')

        # Add cylinder.
        cylinder_size = (0.02, 0.02, 0.08)  # x, y, z dimensions for the asset size
        cylinder_pose = self.get_random_pose(env, cylinder_size)
        cylinder_urdf = self.fill_template('cylinder/cylinder-template.urdf', {'DIM': cylinder_size})
        cylinder_id = env.add_object(cylinder_urdf, cylinder_pose, color=utils.COLORS['blue'])

        # Goal: cylinder is aligned with the line.
        self.add_goal(objs=[cylinder_id], matches=np.ones((1, 1)), targ_poses=[line_pose], replace=False,
                rotations=True, metric='pose', params=None, step_max_reward=1, language_goal=self.lang_template)