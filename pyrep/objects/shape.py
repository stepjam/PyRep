from typing import List, Tuple
from pyrep.backend import vrep
from pyrep.objects.object import Object
from pyrep.const import ObjectType, PrimitiveShape, TextureMappingMode
from pyrep.textures.texture import Texture
import os


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

        handle = vrep.simCreatePureShape(type.value, options, size, mass, None)
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
        verticies, indices, names = vrep.simImportMesh(
            0, filename, options, 0, scaling_factor)
        mesh_objects = []
        for v, i, n in zip(verticies, indices, names):
            mesh_ob = Shape.create_mesh(v, i)
            mesh_objects.append(mesh_ob)
        grouped = mesh_objects[0]
        if len(mesh_objects) > 1:
            handles = [o.get_handle() for o in mesh_objects]
            handle = vrep.simGroupShapes(handles)
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
        handle = vrep.simCreateMeshShape(
            options, shading_angle, vertices, indices)
        return Shape(handle)

    def get_type(self) -> ObjectType:
        return ObjectType.SHAPE

    def is_respondable(self) -> bool:
        """Whether the shape is respondable or not.

        :return: If the shape is respondable.
        """
        return vrep.simGetObjectInt32Parameter(
            self._handle, vrep.sim_shapeintparam_respondable)

    def set_respondable(self, value: bool) -> None:
        """Set whether the shape is respondable or not.

        :param value: The new value of the respondable state of the shape.
        """
        vrep.simSetObjectInt32Parameter(
            self._handle, vrep.sim_shapeintparam_respondable, value)
        self.reset_dynamic_object()

    def is_dynamic(self) -> bool:
        """Whether the shape is dynamic or not.

        :return: If the shape is dynamic.
        """
        return not vrep.simGetObjectInt32Parameter(
            self._handle, vrep.sim_shapeintparam_static)

    def set_dynamic(self, value: bool) -> None:
        """Set whether the shape is dynamic or not.

        :param value: The new value of the dynamic state of the shape.
        """
        vrep.simSetObjectInt32Parameter(
            self._handle, vrep.sim_shapeintparam_static, not value)
        self.reset_dynamic_object()

    def get_color(self) -> List[float]:
        """Gets the shape color.

        :return: The r, g, b values of the shape.
        """
        return vrep.simGetShapeColor(
            self._handle, None, vrep.sim_colorcomponent_ambient_diffuse)

    def set_color(self, color: List[float]) -> None:
        """Sets the color of the shape.

        :param color: The r, g, b values of the shape.
        :return:
        """
        vrep.simSetShapeColor(
            self._handle, None, vrep.sim_colorcomponent_ambient_diffuse, color)

    def get_mass(self) -> float:
        """Gets the mass of the shape.

        :return: A float representing the mass.
        """
        return vrep.simGetObjectFloatParameter(self._handle,
                                               vrep.sim_shapefloatparam_mass)

    def set_mass(self, mass: float) -> None:
        """Sets the mass of the shape.

        :param mass: The new mass value.
        """
        vrep.simSetObjectFloatParameter(
            self._handle, vrep.sim_shapefloatparam_mass, mass)

    def get_mesh_data(self) -> Tuple[List[float], List[float], List[float]]:
        """Retrieves a shape's mesh information.

        :return: A tuple containing a list of vertices, indices, and normals.
        """
        return vrep.simGetShapeMesh(self._handle)

    def get_texture(self):
        """Retrieves the texture from the shape.
        :return: The texture associated with this object.
        """
        return Texture(vrep.simGetShapeTextureId(self.get_handle()))

    def remove_texture(self):
        """Removes the texture from the shape.
        """
        vrep.simSetShapeTexture(self.get_handle(), -1, 0, 0, [1,1], None, None)

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
            options |= 3
        if repeat_along_v:
            options |= 4
        vrep.simSetShapeTexture(
            self.get_handle(), texture.get_texture_id(), mapping_mode.value,
            options, list(uv_scaling), position, orientation)


