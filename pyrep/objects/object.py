import warnings

from pyrep.backend import sim_const as simc
from pyrep.backend.sim import SimBackend
from pyrep.const import ObjectType
from pyrep.errors import (
    WrongObjectTypeError,
    ObjectIsNotModelError,
    ObjectAlreadyRemovedError,
)
from typing import Any, Dict, List, Tuple, Union, Optional
import numpy as np


object_type_to_class: Dict[ObjectType, Any] = {}


class Object(object):
    """Base class for V-REP scene objects that are used for building a scene.

    Objects are visible in the scene hierarchy and in the scene view.
    """

    def __init__(
        self, name_or_handle: Union[str, int], index: int = 0, proxy: "Object" = None
    ):
        self._sim_api = SimBackend().sim_api
        if isinstance(name_or_handle, int):
            self._handle = name_or_handle
        else:
            extra = {}
            prefix = "/"
            if index > 0:
                extra["index"] = 0
            if proxy is not None:
                prefix = "./"
                extra["proxy"] = proxy.get_handle()
            try:
                self._handle = self._sim_api.getObject(name_or_handle, extra)
            except Exception:
                try:
                    self._handle = self._sim_api.getObject(
                        prefix + name_or_handle, extra
                    )
                except Exception:
                    raise ValueError(f"Object with name '{name_or_handle}' not found!")
        assert_type = self._get_requested_type()
        actual = ObjectType(self._sim_api.getObjectType(self._handle))
        if actual != assert_type:
            raise WrongObjectTypeError(
                "You requested object of type %s, but the actual type was "
                "%s" % (assert_type.name, actual.name)
            )

    def __eq__(self, other: object):
        if not isinstance(other, Object):
            raise NotImplementedError
        return self.get_handle() == other.get_handle()

    @staticmethod
    def exists(name: str) -> bool:
        """Checks if the given object is in the scene.

        :param id: name/id of object. If the name is appended by a "@alt"
            suffix, then the object handle based on the object's alternative
            name will be retrieved.
        :return: True of the object exists.
        """
        sim_api = SimBackend().sim_api
        handle = -1
        try:
            handle = sim_api.getObject("/" + name)
        except Exception:
            pass
        return handle >= 0

    @staticmethod
    def get_object_type(name_or_handle: Union[int, str]) -> ObjectType:
        """Gets the type of the object.

        :return: Type of the object.
        """
        sim_api = SimBackend().sim_api
        handle = name_or_handle
        if not isinstance(name_or_handle, int):
            handle = sim_api.getObject("/" + name_or_handle)
        return ObjectType(sim_api.getObjectType(handle))

    @staticmethod
    def get_object_name(name_or_handle: Union[str, int]) -> str:
        """Gets object name.

        :return: Object name.
        """
        sim_api = SimBackend().sim_api
        if isinstance(name_or_handle, int):
            name = sim_api.getObjectName(name_or_handle)
        else:
            name = name_or_handle
        return name

    @staticmethod
    def get_object(name_or_handle: Union[int, str]) -> "Object":
        """Gets object retrieved by name.

        :return: The object.
        """
        object_type = Object.get_object_type(name_or_handle)
        cls = object_type_to_class[object_type]
        return cls(name_or_handle)

    def _get_requested_type(self) -> ObjectType:
        """Used for internally checking assumptions user made about object type.

        :return: Type of the object.
        """
        raise NotImplementedError("Must be overridden.")

    def get_type(self) -> ObjectType:
        """Gets the type of the object.

        :return: Type of the object.
        """
        return ObjectType(self._sim_api.getObjectType(self._handle))

    def get_handle(self) -> int:
        """Gets the internal handle of this object.

        :return: The internal handle.
        """
        return self._handle

    def still_exists(self) -> bool:
        """Gets whether this object is still in the scene or not.

        :return: Whether the object exists or not.
        """
        return self._sim_api.isHandle(self._handle)

    def get_name(self) -> str:
        """Gets the objects name in the scene.

        :return: The objects name.
        """
        return self._sim_api.getObjectAlias(self._handle, -1)

    def set_name(self, name: str) -> None:
        """Sets the objects name in the scene."""
        self._sim_api.setObjectAlias(self._handle, name)

    def scale_object(self, scale_x: float, scale_y: float, scale_z: float) -> None:
        """Scales the object by the given amounts in the x, y, z axes

        Note that this function apply the given scale to the object, it doesn't
        set the scale factors. This means that if you apply two calls to this
        function with factors of 2.0, this results in the object being scaled
        by a factor of 4 in total (it doesn't set an internal scale factor!)

        :param scale_x: The scaling factor along the object's x axis
        :param scale_y: The scaling factor along the object's y axis
        :param scale_z: The scaling factor along the object's z axis
        """
        self._sim_api.scaleObject(self._handle, scale_x, scale_y, scale_z)

    def get_size_factor(self) -> float:
        """Gets the object size factor (for scaling purposes)

        :return: The object's size factor
        """
        return self._sim_api.getObjectSizeFactor(self._handle)

    def get_position(self, relative_to=None) -> np.ndarray:
        """Gets the position of this object.

        :param relative_to: Indicates relative to which reference frame we want
            the position. Specify None to retrieve the absolute position, or an
            Object relative to whose reference frame we want the position.
        :return: An array containing the x, y, z position of the object.
        """
        relto = -1 if relative_to is None else relative_to.get_handle()
        position = self._sim_api.getObjectPosition(self._handle, relto)
        return np.array(position, dtype=np.float64)

    def set_position(
        self, position: Union[list, np.ndarray], relative_to=None, reset_dynamics=True
    ) -> None:
        """Sets the position of this object.

        :param position: A list containing the x, y, z position of the object.
        :param relative_to: Indicates relative to which reference frame the
            the position is specified. Specify None to set the absolute
            position, or an Object relative to whose reference frame the
            position is specified.
        :param reset_dynamics: If we want to reset the dynamics when moving
            an object instantaneously.
        """
        relto = -1 if relative_to is None else relative_to.get_handle()
        if reset_dynamics:
            for ob in self.get_objects_in_tree(exclude_base=False):
                ob.reset_dynamic_object()

        self._sim_api.setObjectPosition(self._handle, relto, list(position))

    def get_orientation(self, relative_to=None) -> np.ndarray:
        """Gets the orientation of this object.

        :param relative_to: Indicates relative to which reference frame we want
            the orientation. Specify None to retrieve the absolute orientation,
            or an Object relative to whose reference frame we want the
            orientation.
        :return: A list containing the x, y, z orientation of the
            object (in radians).
        """
        relto = -1 if relative_to is None else relative_to.get_handle()
        orientation = self._sim_api.getObjectOrientation(self._handle, relto)
        return np.array(orientation, dtype=np.float64)

    def set_orientation(
        self,
        orientation: Union[list, np.ndarray],
        relative_to=None,
        reset_dynamics=True,
    ) -> None:
        """Sets the orientation of this object.

        :param orientation: An array containing the x, y, z orientation of
            the object (in radians).
        :param relative_to: Indicates relative to which reference frame the
            the orientation is specified. Specify None to set the absolute
            orientation, or an Object relative to whose reference frame the
            orientation is specified.
        :param reset_dynamics: If we want to reset the dynamics when rotating
            an object instantaneously.
        """
        relto = -1 if relative_to is None else relative_to.get_handle()
        if reset_dynamics:
            for ob in self.get_objects_in_tree(exclude_base=False):
                ob.reset_dynamic_object()
        self._sim_api.setObjectOrientation(self._handle, relto, list(orientation))

    def get_quaternion(self, relative_to=None) -> np.ndarray:
        """Retrieves the quaternion (x,y,z,w) of an object.

        :param relative_to: Indicates relative to which reference frame we want
            the orientation. Specify None to retrieve the absolute orientation,
            or an Object relative to whose reference frame we want the
            orientation.
        :return: A list containing the quaternion (x,y,z,w).
        """
        relto = -1 if relative_to is None else relative_to.get_handle()
        quaternion = self._sim_api.getObjectQuaternion(self._handle, relto)
        return np.array(quaternion, dtype=np.float64)

    def set_quaternion(
        self, quaternion: Union[list, np.ndarray], relative_to=None, reset_dynamics=True
    ) -> None:
        """Sets the orientation of this object.

        If the quaternion is not normalised, it will be normalised for you.

        :param quaternion: An array containing the quaternion (x,y,z,w).
        :param relative_to: Indicates relative to which reference frame the
            the orientation is specified. Specify None to set the absolute
            orientation, or an Object relative to whose reference frame the
            orientation is specified.
        :param reset_dynamics: If we want to reset the dynamics when rotating
            an object instantaneously.
        """
        assert len(quaternion) == 4
        quaternion = np.asarray(quaternion)
        norm = np.linalg.norm(quaternion)
        if norm != 1.0:
            quaternion = quaternion / norm
        relto = -1 if relative_to is None else relative_to.get_handle()
        if reset_dynamics:
            for ob in self.get_objects_in_tree(exclude_base=False):
                ob.reset_dynamic_object()
        self._sim_api.setObjectQuaternion(self._handle, relto, list(quaternion))

    def get_pose(self, relative_to=None) -> np.ndarray:
        """Retrieves the position and quaternion of an object

        :param relative_to: Indicates relative to which reference frame we want
            the pose. Specify None to retrieve the absolute pose, or an Object
            relative to whose reference frame we want the pose.
        :return: An array containing the (X,Y,Z,Qx,Qy,Qz,Qw) pose of
            the object.
        """
        position = self.get_position(relative_to)
        quaternion = self.get_quaternion(relative_to)
        return np.r_[position, quaternion]

    def set_pose(
        self, pose: Union[list, np.ndarray], relative_to=None, reset_dynamics=True
    ) -> None:
        """Sets the position and quaternion of an object.

        :param pose: An array containing the (X,Y,Z,Qx,Qy,Qz,Qw) pose of
            the object.
        :param relative_to: Indicates relative to which reference frame the
            the pose is specified. Specify None to set the absolute pose, or an
            Object relative to whose reference frame the pose is specified.
        :param reset_dynamics: If we want to reset the dynamics when rotating
            an object instantaneously.
        """
        assert len(pose) == 7
        self.set_position(pose[:3], relative_to, reset_dynamics)
        self.set_quaternion(pose[3:], relative_to, reset_dynamics)

    def get_velocity(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the velocity of this object.

        :return: A pair of linear and angular velocity.
        """
        linear_vel, angular_vel = self._sim_api.getObjectVelocity(self._handle)
        linear_vel = np.array(linear_vel, dtype=np.float64)
        angular_vel = np.array(angular_vel, dtype=np.float64)
        return linear_vel, angular_vel

    def get_parent(self) -> Union["Object", None]:
        """Gets the parent of this object in the scene hierarchy.

        :return: The parent of this object, or None if it doesn't have a parent.
        """
        handle = self._sim_api.getObjectParent(self._handle)
        if handle < 0:
            return None
        object_type = ObjectType(self._sim_api.getObjectType(handle))
        cls = object_type_to_class.get(object_type, Object)
        return cls(handle)

    def set_parent(
        self, parent_object: Union["Object", None], keep_in_place=True
    ) -> None:
        """Sets this objects parent object in the scene hierarchy.

        :param parent_object: The object that will become parent, or None if
            the object should become parentless.
        :param keep_in_place: Indicates whether the object's absolute position
            and orientation should stay same
        """
        parent = -1 if parent_object is None else parent_object.get_handle()
        self._sim_api.setObjectParent(self._handle, parent, keep_in_place)

    def get_matrix(self, relative_to=None) -> np.ndarray:
        """Retrieves the transformation matrix of this object.

        :param relative_to: Indicates relative to which reference frame we want
            the matrix. Specify None to retrieve the absolute transformation
            matrix, or an Object relative to whose reference frame we want the
            transformation matrix.
        :return: A 4x4 transformation matrix.
        """
        relto = -1 if relative_to is None else relative_to.get_handle()
        m = self._sim_api.getObjectMatrix(self._handle, relto)
        m_np = np.array(m).reshape((3, 4))
        return np.concatenate([m_np, [np.array([0, 0, 0, 1])]])

    def set_matrix(self, matrix: np.ndarray, relative_to=None) -> None:
        """Sets the transformation matrix of this object.

        :param relative_to: Indicates relative to which reference frame the
            matrix is specified. Specify None to set the absolute transformation
            matrix, or an Object relative to whose reference frame the
            transformation matrix is specified.
        :param matrix: A 4x4 transformation matrix.
        """
        if not isinstance(matrix, np.ndarray):
            raise ValueError("Expected Numpy 4x4 array.")
        relto = -1 if relative_to is None else relative_to.get_handle()
        self._sim_api.setObjectMatrix(
            self._handle, relto, matrix[:3, :4].reshape((12)).tolist()
        )

    def is_collidable(self) -> bool:
        """Whether the object is collidable or not.

        :return: If the object is collidable.
        """
        return self._get_property(simc.sim_objectspecialproperty_collidable)

    def set_collidable(self, value: bool) -> None:
        """Set whether the object is collidable or not.

        :param value: The new value of the collidable state.
        """
        self._set_property(simc.sim_objectspecialproperty_collidable, value)

    def get_contact(
        self, contact_obj: Optional["Object"]
    ) -> Union[tuple[list, list, list], None]:
        """Get the contact point and force with other object

        :param contact_obj: The object want to check contact info with.
        :return: a tuple containing contact point, force, and normal.
        """
        contact_index = 0
        contact_obj_handle = None if contact_obj is None else contact_obj.get_handle()
        while True:
            handles, point, force, norm_contact_point = self._sim_api.getContactInfo(
                simc.sim_handle_all, self.get_handle(), contact_index
            )
            if len(handles) == 0:
                return None
            contact_index += 1
            if contact_obj_handle is not None and contact_obj_handle in handles:
                return point, force, norm_contact_point

    def get_contacts(self) -> list[tuple[list, list, list]]:
        """Get the contact point and force with all objects

        :return: A list of tuples containing contact point, force, and normal.
        """
        contact_index = 0
        contact_info = []
        while True:
            handles, point, force, norm_contact_point = self._sim_api.getContactInfo(
                simc.sim_handle_all, self.get_handle(), contact_index
            )
            if len(handles) == 0:
                break
            contact_index += 1
            contact_info.append((point, force, norm_contact_point))
        return contact_info

    def is_measurable(self) -> bool:
        """Whether the object is measurable or not.

        :return: If the object is measurable.
        """
        return self._get_property(simc.sim_objectspecialproperty_measurable)

    def set_measurable(self, value: bool):
        """Set whether the object is measurable or not.

        :param value: The new value of the measurable state.
        """
        self._set_property(simc.sim_objectspecialproperty_measurable, value)

    def is_detectable(self) -> bool:
        """Whether the object is detectable or not.

        :return: If the object is detectable.
        """
        return self._get_property(simc.sim_objectspecialproperty_detectable_all)

    def set_detectable(self, value: bool):
        """Set whether the object is detectable or not.

        :param value: The new value of the detectable state.
        """
        self._set_property(simc.sim_objectspecialproperty_detectable_all, value)

    def is_renderable(self) -> bool:
        """Whether the object is renderable or not.

        :return: If the object is renderable.
        """
        return self._get_property(simc.sim_objectspecialproperty_renderable)

    def set_renderable(self, value: bool):
        """Set whether the object is renderable or not.

        :param value: The new value of the renderable state.
        """
        self._set_property(simc.sim_objectspecialproperty_renderable, value)

    def is_model(self) -> bool:
        """Whether the object is a model or not.

        :return: If the object is a model.
        """
        prop = self._sim_api.getModelProperty(self._handle)
        return not (prop & simc.sim_modelproperty_not_model)

    def set_model(self, value: bool):
        """Set whether the object is a model or not.

        :param value: True to set as a model.
        """
        current = self._sim_api.getModelProperty(self._handle)
        current |= simc.sim_modelproperty_not_model
        if value:
            current -= simc.sim_modelproperty_not_model
        self._sim_api.setModelProperty(self._handle, current)

    def remove(self) -> None:
        """Removes this object/model from the scene.

        :raises: ObjectAlreadyRemoved if the object is no longer on the scene.
        """
        try:
            if self.is_model():
                self._sim_api.removeModel(self._handle)
            else:
                self._sim_api.removeObject(self._handle)
        except RuntimeError as e:
            raise ObjectAlreadyRemovedError(
                "The object/model was already deleted."
            ) from e

    def reset_dynamic_object(self) -> None:
        """Dynamically resets an object that is dynamically simulated.

        This means that the object representation in the dynamics engine is
        removed, and added again. This can be useful when the set-up of a
        dynamically simulated chain needs to be modified during simulation
        (e.g. joint or shape attachement position/orientation changed).
        It should be noted that calling this on a dynamically simulated object
        might slightly change its position/orientation relative to its parent
        (since the object will be disconnected from the dynamics world in its
        current position/orientation), so the user is in charge of rectifying
        for that.
        """
        self._sim_api.resetDynamicObject(self._handle)

    def get_bounding_box(self) -> List[float]:
        """Gets the bounding box (relative to the object reference frame).

        :return: A list containing the min x, max x, min y, max y, min z, max z
            positions.
        """
        params = [
            simc.sim_objfloatparam_objbbox_min_x,
            simc.sim_objfloatparam_objbbox_max_x,
            simc.sim_objfloatparam_objbbox_min_y,
            simc.sim_objfloatparam_objbbox_max_y,
            simc.sim_objfloatparam_objbbox_min_z,
            simc.sim_objfloatparam_objbbox_max_z,
        ]
        return [self._sim_api.getObjectFloatParam(self._handle, p) for p in params]

    def get_extension_string(self) -> str:
        """A string that describes additional environment/object properties.

        :return: The extension string.
        """
        val = self._sim_api.getExtensionString(self._handle, -1, "")
        return "" if val is None else val

    def get_configuration_tree(self) -> bytes:
        """Retrieves configuration information for a hierarchy tree.

        Configuration includes object relative positions/orientations,
        joint/path values. Calling :py:meth:`PyRep.set_configuration_tree` at a
        later time, will restore the object configuration
        (use this function to temporarily save object
        positions/orientations/joint/path values).

        :return: The configuration tree.
        """
        return self._sim_api.getConfigurationTree(self._handle)

    def rotate(self, rotation: List[float]) -> None:
        """Rotates a transformation matrix.

        :param rotation: The x, y, z rotation to perform (in radians).
        """
        m = self._sim_api.getObjectMatrix(self._handle, -1)
        x_axis = [m[0], m[4], m[8]]
        y_axis = [m[1], m[5], m[9]]
        z_axis = [m[2], m[6], m[10]]
        axis_pos = self._sim_api.getObjectPosition(self._handle, -1)
        m = self._sim_api.rotateAroundAxis(m, z_axis, axis_pos, rotation[2])
        m = self._sim_api.rotateAroundAxis(m, y_axis, axis_pos, rotation[1])
        m = self._sim_api.rotateAroundAxis(m, x_axis, axis_pos, rotation[0])
        self._sim_api.setObjectMatrix(self._handle, -1, m)

    def check_collision(self, obj: "Object" = None) -> bool:
        """Checks whether two entities are colliding.

        :param obj: The other collidable object to check collision against,
            or None to check against all collidable objects. Note that objects
            must be marked as collidable!
        :return: If the object is colliding.
        """
        handle = simc.sim_handle_all if obj is None else obj.get_handle()
        return self._sim_api.checkCollision(self._handle, handle)[0] == 1

    # === Model specific methods ===

    def is_model_collidable(self) -> bool:
        """Whether the model is collidable or not.

        :raises: ObjectIsNotModel if the object is not a model.
        :return: If the model is collidable.
        """
        return self._get_model_property(simc.sim_modelproperty_not_collidable)

    def set_model_collidable(self, value: bool):
        """Set whether the model is collidable or not.

        :param value: The new value of the collidable state of the model.
        :raises: ObjectIsNotModel if the object is not a model.
        """
        self._set_model_property(simc.sim_modelproperty_not_collidable, value)

    def is_model_measurable(self) -> bool:
        """Whether the model is measurable or not.

        :raises: ObjectIsNotModel if the object is not a model.
        :return: If the model is measurable.
        """
        return self._get_model_property(simc.sim_modelproperty_not_measurable)

    def set_model_measurable(self, value: bool):
        """Set whether the model is measurable or not.

        :param value: The new value of the measurable state of the model.
        :raises: ObjectIsNotModel if the object is not a model.
        """
        self._set_model_property(simc.sim_modelproperty_not_measurable, value)

    def is_model_detectable(self) -> bool:
        """Whether the model is detectable or not.

        :raises: ObjectIsNotModel if the object is not a model.
        :return: If the model is detectable.
        """
        return self._get_model_property(simc.sim_modelproperty_not_detectable)

    def set_model_detectable(self, value: bool):
        """Set whether the model is detectable or not.

        :param value: The new value of the detectable state of the model.
        :raises: ObjectIsNotModel if the object is not a model.
        """
        self._set_model_property(simc.sim_modelproperty_not_detectable, value)

    def is_model_renderable(self) -> bool:
        """Whether the model is renderable or not.

        :raises: ObjectIsNotModel if the object is not a model.
        :return: If the model is renderable.
        """
        return self._get_model_property(simc.sim_modelproperty_not_renderable)

    def set_model_renderable(self, value: bool):
        """Set whether the model is renderable or not.

        :param value: The new value of the renderable state of the model.
        :raises: ObjectIsNotModel if the object is not a model.
        """
        self._set_model_property(simc.sim_modelproperty_not_renderable, value)

    def is_model_dynamic(self) -> bool:
        """Whether the model is dynamic or not.

        :raises: ObjectIsNotModel if the object is not a model.
        :return: If the model is dynamic.
        """
        return self._get_model_property(simc.sim_modelproperty_not_dynamic)

    def set_model_dynamic(self, value: bool):
        """Set whether the model is dynamic or not.

        :param value: The new value of the dynamic state of the model.
        :raises: ObjectIsNotModel if the object is not a model.
        """
        self._set_model_property(simc.sim_modelproperty_not_dynamic, value)

    def is_model_respondable(self) -> bool:
        """Whether the model is respondable or not.

        :raises: ObjectIsNotModel if the object is not a model.
        :return: If the model is respondable.
        """
        return self._get_model_property(simc.sim_modelproperty_not_respondable)

    def set_model_respondable(self, value: bool):
        """Set whether the model is respondable or not.

        :param value: The new value of the respondable state of the model.
        :raises: ObjectIsNotModel if the object is not a model.
        """
        self._set_model_property(simc.sim_modelproperty_not_respondable, value)

    def save_model(self, path: str) -> None:
        """Saves a model.

        Object can be turned to models via :py:meth:`Object.set_model`.
        Any existing file with same name will be overwritten.

        :param path: model filename. The filename extension is required ("ttm").
        :raises: ObjectIsNotModel if the object is not a model.
        """
        self._check_model()
        self._sim_api.saveModel(self._handle, path)

    def get_model_bounding_box(self) -> List[float]:
        """Gets the models bounding box (relative to models reference frame).

        :raises: ObjectIsNotModel if the object is not a model.
        :return: A list containing the min x, max x, min y, max y, min z, max z
            positions.
        """
        self._check_model()
        params = [
            simc.sim_objfloatparam_modelbbox_min_x,
            simc.sim_objfloatparam_modelbbox_max_x,
            simc.sim_objfloatparam_modelbbox_min_y,
            simc.sim_objfloatparam_modelbbox_max_y,
            simc.sim_objfloatparam_modelbbox_min_z,
            simc.sim_objfloatparam_modelbbox_max_z,
        ]
        return [self._sim_api.getObjectFloatParam(self._handle, p) for p in params]

    @staticmethod
    def _get_objects_in_tree(
        root_object=None,
        object_type=ObjectType.ALL,
        exclude_base=True,
        first_generation_only=False,
    ) -> List["Object"]:
        sim_api = SimBackend().sim_api
        if root_object is None:
            root_object = simc.sim_handle_scene
        elif isinstance(root_object, Object):
            root_object = root_object.get_handle()
        elif not isinstance(root_object, int):
            raise ValueError("root_object must be None, int or Object")

        options = 0
        if exclude_base:
            options |= 1
        if first_generation_only:
            options |= 2
        handles = sim_api.getObjectsInTree(root_object, object_type.value, options)
        objects = []
        for handle in handles:
            try:
                objects.append(Object.get_object(handle))
            except KeyError:
                name = Object.get_object_name(handle)
                type = Object.get_object_type(name)
                warnings.warn(
                    "Object ({}, '{}') has {}, "
                    "which is not supported".format(handle, name, type)
                )
        return objects

    def get_objects_in_tree(self, *args, **kwargs) -> List["Object"]:
        """Retrieves the objects in a given hierarchy tree.

        :param object_type: The object type to retrieve.
            One of :py:class:`.ObjectType`.
        :param exclude_base: Exclude the tree base from the returned list.
        :param first_generation_only: Include in the returned list only the
            object's first children. Otherwise, entire hierarchy is returned.
        :return: A list of objects in the hierarchy tree.
        """
        return self._get_objects_in_tree(self._handle, *args, **kwargs)

    def copy(self) -> "Object":
        """Copy and pastes object in the scene.

        The object is copied together with all its associated calculation
        objects and associated scripts.

        :return: The new pasted object.
        """
        return self.__class__((self._sim_api.copyPasteObjects([self._handle], 0)[0]))

    def check_distance(self, other: "Object") -> float:
        """Checks the minimum distance between two objects.

        :param other: The other object to check distance against.
        :return: The distance between the objects.
        """
        result, distance_data, object_handle_pair = self._sim_api.checkDistance(
            self.get_handle(), other.get_handle(), -1
        )
        obj1x, obj1y, obj1z, obj2x, obj2y, obj2z, _ = distance_data
        return np.linalg.norm(
            np.array([obj1x, obj1y, obj1z]) - np.array([obj2x, obj2y, obj2z])
        )

    def get_bullet_friction(self) -> float:
        """Get bullet friction parameter.

        :return: The friction.
        """
        return self._sim_api.getEngineFloatParameter(
            simc.sim_bullet_body_friction, self._handle
        )

    def set_bullet_friction(self, friction) -> None:
        """Set bullet friction parameter.

        :param friction: The friction to set.
        """
        self._sim_api.setEngineFloatParameter(
            simc.sim_bullet_body_friction, self._handle, friction
        )

    def get_explicit_handling(self) -> int:
        """Get explicit handling flags.

        :return: The flag: enabled(1) or disabled(0).
        """
        return self._sim_api.getExplicitHandling(self._handle)

    def set_explicit_handling(self, value: int) -> None:
        """Set explicit handling flags.

        :param value: A flag to enable(1) or disable(0) explicit handling.
        """
        self._sim_api.setExplicitHandling(self._handle, value)

    # === Private methods ===

    def _check_model(self) -> None:
        if not self.is_model():
            raise ObjectIsNotModelError(
                "Object '%s' is not a model. Use 'set_model(True)' to convert."
            )

    def _get_model_property(self, prop_type: int) -> bool:
        current = self._sim_api.getModelProperty(self._handle)
        return (current & prop_type) == 0

    def _set_model_property(self, prop_type: int, value: bool) -> None:
        current = self._sim_api.getModelProperty(self._handle)
        current |= prop_type  # Makes is not X
        if value:
            current -= prop_type
        self._sim_api.setModelProperty(self._handle, current)

    def _get_property(self, prop_type: int) -> bool:
        current = self._sim_api.getObjectSpecialProperty(self._handle)
        return current & prop_type

    def _set_property(self, prop_type: int, value: bool) -> None:
        current = self._sim_api.getObjectSpecialProperty(self._handle)
        current |= prop_type
        if not value:
            current -= prop_type
        self._sim_api.setObjectSpecialProperty(self._handle, current)
