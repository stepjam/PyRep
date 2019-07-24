from pyrep.backend import vrep
from enum import Enum


class PrimitiveShape(Enum):
    CUBOID = 0
    SPHERE = 1
    CYLINDER = 2
    CONE = 3


class ObjectType(Enum):
    ALL = vrep.sim_handle_all
    SHAPE = vrep.sim_object_shape_type
    JOINT = vrep.sim_object_joint_type
    DUMMY = vrep.sim_object_dummy_type
    PROXIMITY_SENSOR = vrep.sim_object_proximitysensor_type
    GRAPH = vrep.sim_object_graph_type
    CAMERA = vrep.sim_object_camera_type
    PATH = vrep.sim_object_path_type
    VISION_SENSOR = vrep.sim_object_visionsensor_type
    VOLUME = vrep.sim_object_volume_type
    MILl = vrep.sim_object_mill_type
    FORCE_SENSOR = vrep.sim_object_forcesensor_type
    LIGHT = vrep.sim_object_light_type
    MIRROR = vrep.sim_object_mirror_type


class JointType(Enum):
    REVOLUTE = vrep.sim_joint_revolute_subtype
    PRISMATIC = vrep.sim_joint_prismatic_subtype
    SPHERICAL = vrep.sim_joint_spherical_subtype


class JointMode(Enum):
    PASSIVE = vrep.sim_jointmode_passive
    IK = vrep.sim_jointmode_ik
    IK_DEPENDENT = vrep.sim_jointmode_ikdependent
    DEPENDENT = vrep.sim_jointmode_dependent
    FORCE = vrep.sim_jointmode_force


class ConfigurationPathAlgorithms(Enum):
    BiTRRT = 'BiTRRT'
    BITstar = 'BITstar'
    BKPIECE1 = 'BKPIECE1'
    CForest = 'CForest'
    EST = 'EST'
    FMT = 'FMT'
    KPIECE1 = 'KPIECE1'
    LazyPRM = 'LazyPRM'
    LazyPRMstar = 'LazyPRMstar'
    LazyRRT = 'LazyRRT'
    LBKPIECE1 = 'LBKPIECE1'
    LBTRRT = 'LBTRRT'
    PDST = 'PDST'
    PRM = 'PRM'
    PRMstar = 'PRMstar'
    pRRT = 'pRRT'
    pSBL = 'pSBL'
    RRT = 'RRT'
    RRTConnect = 'RRTConnect'
    RRTstar = 'RRTstar'
    SBL = 'SBL'
    SPARS = 'SPARS'
    SPARStwo = 'SPARStwo'
    STRIDE = 'STRIDE'
    TRRT = 'TRRT'


class TextureMappingMode(Enum):
    PLANE = vrep.sim_texturemap_plane
    CYLINDER = vrep.sim_texturemap_cylinder
    SPHERE = vrep.sim_texturemap_sphere
    CUBE = vrep.sim_texturemap_cube


class PerspectiveMode(Enum):
    ORTHOGRAPHIC = 0
    PERSPECTIVE = 1


class RenderMode(Enum):
    OPENGL = vrep.sim_rendermode_opengl
    OPENGL_AUXILIARY = vrep.sim_rendermode_auxchannels
    OPENGL_COLOR_CODED = vrep.sim_rendermode_colorcoded
    POV_RAY = vrep.sim_rendermode_povray
    EXTERNAL = vrep.sim_rendermode_extrenderer
    EXTERNAL_WINDOWED = vrep.sim_rendermode_extrendererwindowed
    OPENGL3 = vrep.sim_rendermode_opengl3
    OPENGL3_WINDOWED = vrep.sim_rendermode_opengl3windowed


PYREP_SCRIPT_TYPE = vrep.sim_scripttype_addonscript
