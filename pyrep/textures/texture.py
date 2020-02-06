class Texture(object):
    """Base class for V-REP textures."""

    def __init__(self, texture_id: int):
        self._texture_id = texture_id

    def __eq__(self, other: object):
        if not isinstance(other, Texture):
            raise NotImplementedError
        return self.get_texture_id() == other.get_texture_id()

    def get_texture_id(self) -> int:
        """Gets the texture id.

        :return: The internal texture id.
        """
        return self._texture_id

