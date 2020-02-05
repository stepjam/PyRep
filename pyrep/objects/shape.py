from typing import List, Tuple
import numpy as np
from pyrep.backend import sim
from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType, PrimitiveShape, TextureMappingMode
from pyrep.textures.texture import Texture
import os
import collections


SShapeVizInfo = collections.namedtuple(
    'SShapeVizInfo',
    [
        'vertices',
        'indices',
        'normals',
        'shading_angle',
        'colors',
        'texture',
        'texture_id',
        'texture_coords',
        'texture_apply_mode',
        'texture_options',
    ],
)


class Shape(Object):
    """Shapes are rigid mesh objects that are composed of triangular faces.
    """

    @staticmethod
    def create(type: PrimitiveShape, size: List[float],
               mass=1., backface_culling=False, visible_edges=False,
               smooth=False, respondable=True,
               static=False, renderable=True, position=None,
               orientation=None, color=None) -> 'Shape':
        """Creates a primitive shape in the scene.

        :param type: The type of primitive to shape. One of:
            PrimitiveShape.CUBOID
            PrimitiveShape.SPHERE
            PrimitiveShape.CYLINDER
            PrimitiveShape.CONE
        :param size: A list of the x, y, z dimensions.
        :param mass: A float representing the mass of the object.
        :param backface_culling: If backface culling is enabled.
        :param visible_edges: If the object will have visible edges.
        :param smooth: If the shape appears smooth.
        :param respondable: Shape is responsible.
        :param static: If the shape is static.
        :param renderable: If the shape is renderable.
        :param position: The x, y, z position.
        :param orientation: The z, y, z orientation (in radians).
        :param color: The r, g, b values of the shape.
        :return: The created Shape object.
        """
        options = 0
        if backface_culling:
            options |= 1
        if visible_edges:
            options |= 2
        if smooth:
            options |= 4
        if respondable:
            options |= 8
        if static:
            options |= 16

        handle = sim.simCreatePureShape(type.value, options, size, mass, None)
        ob = Shape(handle)
        ob.set_renderable(renderable)
        if position is not None:
            ob.set_position(position)
        if orientation is not None:
            ob.set_orientation(orientation)
        if color is not None:
            ob.set_color(color)
        return ob

    @staticmethod
    def import_mesh(filename: str, scaling_factor=1.0,
                    keep_identical_vertices=False,
                    ignore_up_vector=False) -> 'Shape':
        """Imports a mesh from a file.

        :param filename: The location of the file to import.
        :param scaling_factor: The scaling factor to apply to the imported vertices
        :param keep_identical_vertices: Keep identical vertices.
        :param ignore_up_vector: Ignore up-vector coded in file.
        :return: The grouped Shape object.
        """

        if not os.path.isfile(filename):
            raise ValueError('Filename does not exist: ' + filename)

        options = 0
        if keep_identical_vertices:
            options |= 1
        if ignore_up_vector:
            options |= 128

        # fileformat is 0 as it is automatically detected.
        # identicalVerticeTolerance has no effect. Setting to zero.
        verticies, indices, names = sim.simImportMesh(
            0, filename, options, 0, scaling_factor)
        mesh_objects = []
        for v, i, n in zip(verticies, indices, names):
            mesh_ob = Shape.create_mesh(v, i)
            mesh_objects.append(mesh_ob)
        grouped = mesh_objects[0]
        if len(mesh_objects) > 1:
            handles = [o.get_handle() for o in mesh_objects]
            handle = sim.simGroupShapes(handles)
            grouped = Shape(handle)
        return grouped

    @staticmethod
    def create_mesh(vertices: List[float], indices: List[int],
                    shading_angle=None, backface_culling=False,
                    visible_edges=False) -> 'Shape':
        """Creates a mesh shape.

        :param vertices: A list of vertices.
        :param indices: A list of indices.
        :param shading_angle: The shading angle (in radians).
        :param backface_culling: To enable backface culling.
        :param visible_edges: To enable visible edges.
        :return: The newly created mesh.
        """
        options = 0
        if backface_culling:
            options |= 1
        if visible_edges:
            options |= 2
        if shading_angle is None:
            shading_angle = 20.0 * 3.1415 / 180.0
        handle = sim.simCreateMeshShape(
            options, shading_angle, vertices, indices)
        return Shape(handle)

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.SHAPE

    def is_respondable(self) -> bool:
        """Whether the shape is respondable or not.

        :return: If the shape is respondable.
        """
        return sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_shapeintparam_respondable)

    def set_respondable(self, value: bool) -> None:
        """Set whether the shape is respondable or not.

        :param value: The new value of the respondable state of the shape.
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_shapeintparam_respondable, value)
        self.reset_dynamic_object()

    def is_dynamic(self) -> bool:
        """Whether the shape is dynamic or not.

        :return: If the shape is dynamic.
        """
        return not sim.simGetObjectInt32Parameter(
            self._handle, sim.sim_shapeintparam_static)

    def set_dynamic(self, value: bool) -> None:
        """Set whether the shape is dynamic or not.

        :param value: The new value of the dynamic state of the shape.
        """
        sim.simSetObjectInt32Parameter(
            self._handle, sim.sim_shapeintparam_static, not value)
        self.reset_dynamic_object()

    def get_color(self) -> List[float]:
        """Gets the shape color.

        :return: The r, g, b values of the shape.
        """
        return sim.simGetShapeColor(
            self._handle, None, sim.sim_colorcomponent_ambient_diffuse)

    def set_color(self, color: List[float]) -> None:
        """Sets the color of the shape.

        :param color: The r, g, b values of the shape.
        :return:
        """
        sim.simSetShapeColor(
            self._handle, None, sim.sim_colorcomponent_ambient_diffuse, color)

    def get_mass(self) -> float:
        """Gets the mass of the shape.

        :return: A float representing the mass.
        """
        return sim.simGetObjectFloatParameter(self._handle,
                                              sim.sim_shapefloatparam_mass)

    def set_mass(self, mass: float) -> None:
        """Sets the mass of the shape.

        :param mass: The new mass value.
        """
        sim.simSetObjectFloatParameter(
            self._handle, sim.sim_shapefloatparam_mass, mass)

    def get_mesh_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Retrieves a shape's mesh information.

        :param asnumpy: A flag to cast vertices as numpy array with reshape.
        :return: A tuple containing a list of vertices, indices, and normals.
        """
        vertices, indices, normals = sim.simGetShapeMesh(self._handle)
        vertices = np.array(vertices, dtype=np.float64).reshape(-1, 3)
        indices = np.array(indices, dtype=np.int64).reshape(-1, 3)
        normals = np.array(normals, dtype=np.float64).reshape(-1, 3)
        return vertices, indices, normals

    def get_convex_decomposition(self, morph=False, same=False, use_vhacd=False,
                                 individual_meshes=False,
                                 hacd_extra_points=True, hacd_face_points=True,
                                 hacd_min_clusters=1, hacd_tri_target=500,
                                 hacd_max_vertex=200, hacd_max_iter=4,
                                 hacd_max_concavity=100, hacd_max_dist=30,
                                 hacd_cluster_thresh=0.25,
                                 vhacd_pca=False, vhacd_tetrahedron=False,
                                 vhacd_res=100000, vhacd_depth=20,
                                 vhacd_plane_downsample=4,
                                 vhacd_hull_downsample=4,
                                 vhacd_max_vertex=64, vhacd_concavity=0.0025,
                                 vhacd_alpha=0.05, vhacd_beta=0.05,
                                 vhacd_gamma=0.00125, vhacd_min_vol=0.0001
                                 ) -> 'Shape':
        """
        Compute the convex decomposition of the shape using HACD or V-HACD
         algorithms
        :param morph: The shape will be morphed into its convex decomposition.
            Otherwise a new shape will be created.
        :param same: Use the same parameters as the last call to the function.
        :param use_vhacd: Use V-HACD algorithm.
        :param individual_meshes: Each individual mesh of a compound shape will
            be handled on its own during decomposition, otherwise the
            compound shape is considered as a single mesh.
        :param hacd_extra_points: HACD: Extra points will be added when
            computing the concavity.
        :param hacd_face_points: HACD: Faces points will be added when computing
            the concavity.
        :param hacd_min_clusters: HACD: Minimum number of clusters to generate.
        :param hacd_tri_target: HACD: Targeted number of triangles of the
            decimated mesh.
        :param hacd_max_vertex: HACD: Maximum number of vertices for each
            generated convex hull.
        :param hacd_max_iter: HACD: Maximum number of iterations.
        :param hacd_max_concavity: HACD: The maximum allowed concavity.
        :param hacd_max_dist: HACD: The maximum allowed distance to get convex
            clusters connected.
        :param hacd_cluster_thresh: HACD: The threshold to detect small
            clusters, expressed as a fraction of the total mesh surface.
        :param vhacd_pca: V-HACD: Enable PCA.
        :param vhacd_tetrahedron: V-HACD: Tetrahedron-based approximate convex
            decomposition.  Otherwise, voxel-based decomposition is used.
        :param vhacd_res: V-HACD: Resolution (10000-64000000)
        :param vhacd_depth: V-HACD: Depth (1-32)
        :param vhacd_plane_downsample: V-HACD: Plane downsampling (1-16)
        :param vhacd_hull_downsample: V-HACD: Convex hull downsampling (1-16)
        :param vhacd_max_vertex: V-HACD: Maximum number of vertices per convex
            hull (4-1024)
        :param vhacd_concavity: V-HACD: Concavity (0.0-1.0)
        :param vhacd_alpha: V-HACD: Alpha (0.0-1.0)
        :param vhacd_beta: V-HACD: Beta (0.0-1.0)
        :param vhacd_gamma: V-HACD: Gamma (0.0-1.0)
        :param vhacd_min_vol: V-HACD: Minimum volume per convex hull (0.0-0.01)
        :return: Convex Decomposition of the shape.
        """
        options = 0
        if morph:
            options |= 1
        if same:
            options |= 4
        if hacd_extra_points:
            options |= 8
        if hacd_face_points:
            options |= 16
        if individual_meshes:
            options |= 32
        if use_vhacd:
            options |= 128
        if vhacd_pca:
            options |= 256
        if vhacd_tetrahedron:
            options |= 512

        int_params = [
            hacd_min_clusters,          # [0]
            hacd_tri_target,            # [1]
            hacd_max_vertex,            # [2]
            hacd_max_iter,              # [3]
            0,                          # [4]
            vhacd_res,                  # [5]
            vhacd_depth,                # [6]
            vhacd_plane_downsample,     # [7]
            vhacd_hull_downsample,      # [8]
            vhacd_max_vertex            # [9]
        ]

        float_params = [
            hacd_max_concavity,         # [0]
            hacd_max_dist,              # [1]
            hacd_cluster_thresh,        # [2]
            0.0,                        # [3]
            0.0,                        # [4]
            vhacd_concavity,            # [5]
            vhacd_alpha,                # [6]
            vhacd_beta,                 # [7]
            vhacd_gamma,                # [8]
            vhacd_min_vol               # [9]
        ]

        return Shape(sim.simConvexDecompose(self.get_handle(), options,
                                            int_params, float_params))

    def get_texture(self):
        """Retrieves the texture from the shape.
        :return: The texture associated with this object.
        """
        return Texture(sim.simGetShapeTextureId(self.get_handle()))

    def remove_texture(self):
        """Removes the texture from the shape.
        """
        sim.simSetShapeTexture(self.get_handle(), -1, 0, 0, [1, 1], None, None)

    def set_texture(self, texture: Texture, mapping_mode: TextureMappingMode,
                    interpolate=True, decal_mode=False, repeat_along_u=False,
                    repeat_along_v=False, uv_scaling=[1., 1.],
                    position: List[float] = None,
                    orientation: List[float] = None):
        """Applies a texture to a shape

        :param texture: The texture to add.
        :param mapping_mode: The texture mapping mode. One of:
            TextureMappingMode.PLANE
            TextureMappingMode.CYLINDER
            TextureMappingMode.SPHERE
            TextureMappingMode.CUBE
        :param interpolate: Adjacent texture pixels are not interpolated.
        :param decal_mode: Texture is applied as a decal (its appearance
            won't be influenced by light conditions).
        :param repeat_along_u: Texture will be repeated along the U direction.
        :param repeat_along_v: Texture will be repeated along the V direction.
        :param uv_scaling: A list of 2 values containig the texture scaling
            factors along the U and V directions.
        :param position: A list of (x,y,z) values that indicate the texture
            position on the shape. Can be None for default.
        :param orientation: A list of 3 Euler angles that indicate the texture
            orientation on the shape. Can be None for default.
        """
        options = 0
        if not interpolate:
            options |= 1
        if decal_mode:
            options |= 2
        if repeat_along_u:
            options |= 4
        if repeat_along_v:
            options |= 8
        sim.simSetShapeTexture(
            self.get_handle(), texture.get_texture_id(), mapping_mode.value,
            options, list(uv_scaling), position, orientation)

    def ungroup(self) -> List['Shape']:
        """Ungroups a compound shape into several simple shapes.

        :return: A list of shapes.
        """
        handles = sim.simUngroupShape(self.get_handle())
        return [Shape(handle) for handle in handles]

    def apply_texture(self, texture_coords: np.ndarray, texture: np.ndarray,
                      interpolate: bool = True, decal_mode: bool = False,
                      is_rgba: bool = False, fliph: bool = False,
                      flipv: bool = False) -> None:
        """Apply texture to the shape.

        :param texture_coords: A list of (u, v) values that indicate the
            vertex position on the shape. For each of the shape's triangle,
            there should be exactly 3 UV texture coordinate pairs
        :param texture: The RGB or RGBA texture.
        :param interpolate: A flag to interpolate adjacent texture pixels.
        :param decal_mode: Texture is applied as a decal (its appearance
            won't be influenced by light conditions).
        :param is_rgba: A flag to use RGBA texture.
        :param fliph: A flag to flip texture horizontally.
        :param flipv: A flag to flip texture vertically. Note that CoppeliaSim
            texture coordinates are flipped vertically compared with Pillow
            and OpenCV and this flag must be true in general.
        """
        texture_coords = np.asarray(texture_coords)
        if not isinstance(texture, np.ndarray):
            raise TypeError('texture must be np.ndarray type')
        height, width = texture.shape[:2]

        options = 0
        if not interpolate:
            options |= 1
        if decal_mode:
            options |= 2
        if is_rgba:
            options |= 16
        if fliph:
            options |= 32
        if flipv:
            options |= 64

        sim.simApplyTexture(
            self._handle,
            textureCoordinates=texture_coords.flatten().tolist(),
            textCoordSize=texture_coords.size,
            texture=texture.flatten().tolist(),
            textureResolution=(width, height),
            options=options,
        )

    def get_shape_viz(self, index):
        """Retrieves a shape's visual information.

        :param index: 0-based index of the shape element to retrieve
            (compound shapes contain more than one shape element)

        :return: SShapeVizInfo.
        """
        info = sim.simGetShapeViz(shapeHandle=self._handle, index=index)

        vertices = np.array(info.vertices, dtype=float).reshape(-1, 3)
        indices = np.array(info.indices, dtype=float).reshape(-1, 3)
        normals = np.array(info.normals, dtype=float).reshape(-1, 3)
        colors = np.array(info.colors, dtype=float)
        texture = np.array(info.texture, dtype=np.uint8).reshape(
            info.textureRes[1], info.textureRes[0], 4)
        textureCoords = np.array(info.textureCoords, dtype=float).reshape(
            -1, 2)

        res = SShapeVizInfo(
            vertices=vertices,
            indices=indices,
            normals=normals,
            shading_angle=info.shadingAngle,
            colors=colors,
            texture=texture,
            texture_id=info.textureId,
            texture_coords=textureCoords,
            texture_apply_mode=info.textureApplyMode,
            texture_options=info.textureOptions,
        )
        return res


object_type_to_class[ObjectType.SHAPE] = Shape
