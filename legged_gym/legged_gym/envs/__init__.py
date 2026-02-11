from legged_gym import LEGGED_GYM_ROOT_DIR, LEGGED_GYM_ENVS_DIR
from legged_gym.utils.task_registry import task_registry

from .base.host_ground import LeggedRobot as LeggedRobotGround
from legged_gym.envs.g1.g1_config_ground import G1Cfg as G1CfgGround
from legged_gym.envs.g1.g1_config_ground import G1CfgPPO as G1CfgPPOGround

from .base.host_platform import LeggedRobot as LeggedRobotPlatform
from legged_gym.envs.g1.g1_config_platform import G1Cfg as G1CfgPlatform
from legged_gym.envs.g1.g1_config_platform import G1CfgPPO as G1CfgPPOPlatform

from .base.host_wall import LeggedRobot as LeggedRobotWall
from legged_gym.envs.g1.g1_config_wall import G1Cfg as G1WallCfgWall
from legged_gym.envs.g1.g1_config_wall import G1CfgPPO as G1WallCfgPPOWall

from .base.host_slope import LeggedRobot as LeggedRobotSlope
from legged_gym.envs.g1.g1_config_slope import G1Cfg as G1CfgSlope
from legged_gym.envs.g1.g1_config_slope import G1CfgPPO as G1CfgPPOSlope

from .base.host_ground_prone import LeggedRobot as LeggedRobotGroundProne
from legged_gym.envs.g1.g1_config_ground_prone import G1Cfg as G1CfgGroundProne
from legged_gym.envs.g1.g1_config_ground_prone import G1CfgPPO as G1CfgPPOGroundProne

from legged_gym.envs.h1.h1_config_ground import H1Cfg as H1CfgGround
from legged_gym.envs.h1.h1_config_ground import H1CfgPPO as H1CfgPPOGround

from legged_gym.envs.pi.pi_config_ground import PiCfg as PiCfgGround
from legged_gym.envs.pi.pi_config_ground import PiCfgPPO as PiCfgPPOGround
from legged_gym.envs.pi.pi_host_ground import LeggedRobot_Pi

from legged_gym.envs.taks_t1.taks_t1_config_ground import TaksT1Cfg as TaksT1CfgGround
from legged_gym.envs.taks_t1.taks_t1_config_ground import TaksT1CfgPPO as TaksT1CfgPPOGround
from legged_gym.envs.taks_t1.taks_t1_config_platform import TaksT1Cfg as TaksT1CfgPlatform
from legged_gym.envs.taks_t1.taks_t1_config_platform import TaksT1CfgPPO as TaksT1CfgPPOPlatform
from legged_gym.envs.taks_t1.taks_t1_config_slope import TaksT1Cfg as TaksT1CfgSlope
from legged_gym.envs.taks_t1.taks_t1_config_slope import TaksT1CfgPPO as TaksT1CfgPPOSlope
from legged_gym.envs.taks_t1.taks_t1_config_wall import TaksT1Cfg as TaksT1CfgWall
from legged_gym.envs.taks_t1.taks_t1_config_wall import TaksT1CfgPPO as TaksT1CfgPPOWall
from legged_gym.envs.taks_t1.taks_t1_config_ground_prone import TaksT1Cfg as TaksT1CfgGroundProne
from legged_gym.envs.taks_t1.taks_t1_config_ground_prone import TaksT1CfgPPO as TaksT1CfgPPOGroundProne

task_registry.register( "g1_ground", LeggedRobotGround, G1CfgGround(), G1CfgPPOGround())
task_registry.register( "g1_platform", LeggedRobotPlatform, G1CfgPlatform(), G1CfgPPOPlatform())
task_registry.register( "g1_wall", LeggedRobotWall, G1WallCfgWall(), G1WallCfgPPOWall())
task_registry.register( "g1_slope", LeggedRobotSlope, G1CfgSlope(), G1CfgPPOSlope())
task_registry.register( "g1_ground_prone", LeggedRobotGroundProne, G1CfgGroundProne(), G1CfgPPOGroundProne())
task_registry.register( "h1_ground", LeggedRobotGround, H1CfgGround(), H1CfgPPOGround())

task_registry.register( "pi_ground", LeggedRobot_Pi, PiCfgGround(), PiCfgPPOGround())

task_registry.register( "taks_t1_ground", LeggedRobotGround, TaksT1CfgGround(), TaksT1CfgPPOGround())
task_registry.register( "taks_t1_platform", LeggedRobotPlatform, TaksT1CfgPlatform(), TaksT1CfgPPOPlatform())
task_registry.register( "taks_t1_wall", LeggedRobotWall, TaksT1CfgWall(), TaksT1CfgPPOWall())
task_registry.register( "taks_t1_slope", LeggedRobotSlope, TaksT1CfgSlope(), TaksT1CfgPPOSlope())
task_registry.register( "taks_t1_ground_prone", LeggedRobotGroundProne, TaksT1CfgGroundProne(), TaksT1CfgPPOGroundProne())