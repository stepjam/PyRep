from pyrep.backend.sim import SimBackend
from pyrep.objects.object import Object, object_type_to_class
from pyrep.const import ObjectType
from pyrep.backend import sim


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
        sim_api = SimBackend().sim_api
        handle = sim_api.createDummy(size)
        return Dummy(handle)

    def _get_requested_type(self) -> ObjectType:
        return ObjectType.DUMMY


object_type_to_class[ObjectType.DUMMY] = Dummy
