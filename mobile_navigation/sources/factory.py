from typing import Tuple
from mobile_navigation.sources.sensorlog_handler import SensorLogHandler


class SourceFactory:
    """
    Factory class to create data source handlers for different mobile apps.

    Methods
    -------
    create_source(app_name: str, source: Tuple) -> SensorLogHandler
        Returns an instance of the appropriate source handler.
    """

    @staticmethod
    def create_source(app_name: str, source: Tuple[str, int]) -> SensorLogHandler:
        """
        Creates and returns a source handler object for the specified app.

        Parameters
        ----------
        app_name : str
            Name of the mobile app providing data (e.g., 'sensorlog').
        source : Tuple[str, int]
            Connection info for the data source. For SensorLog, this is (host, port).

        Returns
        -------
        SensorLogHandler
            A handler instance that streams data and provides `get_next()`.

        Raises
        ------
        ValueError
            If the app_name is not supported.
        """
        if app_name.lower() == "sensorlog":
            host, port = source
            return SensorLogHandler(host, port)
        else:
            raise ValueError(f"Unsupported app: {app_name}")
