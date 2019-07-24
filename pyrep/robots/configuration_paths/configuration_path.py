class ConfigurationPath(object):
    """Base class for paths expressed in joint configuration space.
    """

    def step(self) -> bool:
        raise NotImplementedError()

    def set_to_start(self) -> None:
        raise NotImplementedError()

    def set_to_end(self) -> None:
        raise NotImplementedError()

    def visualize(self) -> None:
        raise NotImplementedError()

    def clear_visualization(self) -> None:
        raise NotImplementedError()
