from pyrep.objects.object import Object
from pyrep.const import ObjectType
from pyrep.backend import vrep


class Dummy(Object):
    """A point with orientation.

    Dummies are multipurpose objects that can have many different applications.
    """

    @staticmethod
    def create(size=0.01) -> 'Dummy':
        """Creates a dummy object and inserts in the scene.

        :param size: The size of the dummy object.
        :return: The newly created Dummy.
        """
        handle = vrep.simCreateDummy(size, None)
        return Dummy(handle)

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.DUMMY
