from typing import Union
from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType


class Camera(Object):
    """Cameras can be associated with views.
    """

    def __init__(self, name_or_handle: Union[str, int]):
        super().__init__(name_or_handle)
    
    @staticmethod
    def create():
        raise NotImplementedError("API does not provide simCreateCamera.")

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.CAMERA


object_type_to_class[ObjectType.CAMERA] = Camera
