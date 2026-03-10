"""
Configuration for Dobot Nova5 dual-arm robot system.
Each arm has 6 DOF, with Robotiq 2F-85 grippers as end effectors.
"""
from dataclasses import dataclass, field
from typing import List, Optional

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("dobot_dual_arm")
@dataclass
class DobotDualArmConfig(RobotConfig):
    """Configuration for Dobot Nova5 dual-arm robot with Robotiq 2F-85 grippers."""
    
    # Robot identification
    name: str = "dobot_dual_arm"
    
    # Network configuration - single port for dual-arm control
    robot_ip: str = "localhost"  # dobot server ip
    robot_port: int = 4242  # dual-arm zerorpc port (single port for both arms)
    
    # Gripper configuration (Robotiq 2F-85)
    gripper_ip: str = "localhost"  # gripper zerorpc ip, if different from robot_ip, set to robot_ip
    gripper_port: int = 4243  # gripper zerorpc port (single port for both arms)
    use_gripper: bool = True
    gripper_max_open: float = 0.085  # Robotiq 2F-85 max opening: 85mm
    gripper_force: float = 10.0  # Gripping force in N
    gripper_speed: float = 0.1  # Speed in m/s
    gripper_reverse: bool = False  # Whether to reverse gripper command
    close_threshold: float = 0.5  # Threshold for binary gripper control
    
    # Control configuration
    control_mode: str = "oculus"
    debug: bool = True
    
    # Joint configuration (6 DOF per arm)
    num_joints_per_arm: int = 6
    
    # Cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    
    # Safety limits
    max_joint_velocity: float = 2.0  # rad/s
    max_ee_velocity: float = 0.5  # m/s
    max_joint_delta: float = 0.3  # rad - max joint change per step

