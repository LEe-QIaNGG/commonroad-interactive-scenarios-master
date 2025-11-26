from abc import ABC, abstractmethod
from enum import Enum, auto
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Optional

from typing_extensions import Self, override

from commonroad_sumo.sumolib.net import (
    SumoAdditionalDefinitionsFile,
    SumoConfigFile,
    SumoConnectionsDefinitionsFile,
    SumoEdgeDefinitionsFile,
    SumoEdgeTypeDefinitionsFile,
    SumoNetDefinitionsFile,
    SumoNodeDefinitionsFile,
    SumoRouteDefintionsFile,
    SumoTrafficLightDefinitionsFile,
)
from commonroad_sumo.sumolib.xml import SumoXmlFile


class AbstractSumoProject(ABC):
    """
    Interface to the files of a SUMO project.
    """

    _project_name: str
    """The human readable name of this project. Will be used for filenames."""

    _project_path: Path
    """Path to directory where the project files are located."""

    _temp_dir: TemporaryDirectory[str] | None
    """Binds the lifetime of the temporary directory to the lifetime of this project."""

    def __init__(
        self,
        project_name: str,
        project_path: Path,
        temp_dir: TemporaryDirectory[str] | None = None,
    ) -> None:
        self._project_name = project_name
        self._project_path = project_path

        self._temp_dir = temp_dir

    @classmethod
    def from_folder(cls, project_path: Path) -> Self:
        """
        Create a new `SumoProject` from `project_path`.
        """
        return cls(project_name=project_path.stem, project_path=project_path)

    @classmethod
    def create_temp(cls, project_name: str) -> Self:
        """
        Create a temporary `SumoProject`. The project path is only valid for the lifetime of the created `SumoProject` object.
        """
        temp_dir = TemporaryDirectory()
        return cls(project_name, Path(temp_dir.name), temp_dir)

    @abstractmethod
    def write(self) -> None: ...

    @property
    def project_name(self) -> str:
        return self._project_name

    @property
    def project_path(self) -> Path:
        return self._project_path


class SumoIntermediateFileType(Enum):
    """Collection of the different file types which might be found in an intermediate SUMO project."""

    NODES = auto()
    EDGES = auto()
    CONNECTIONS = auto()
    TLLOGICS = auto()
    TYPES = auto()

    def get_file_extension(self) -> str:
        if self == SumoIntermediateFileType.CONNECTIONS:
            return ".con.xml"
        elif self == SumoIntermediateFileType.NODES:
            return ".nod.xml"
        elif self == SumoIntermediateFileType.EDGES:
            return ".edg.xml"
        elif self == SumoIntermediateFileType.TLLOGICS:
            return ".tll.xml"
        elif self == SumoIntermediateFileType.TYPES:
            return ".typ.xml"
        else:
            raise ValueError(f"Unknown file type: {self}")

    def get_xml_file_type(self) -> type[SumoXmlFile]:
        if self == SumoIntermediateFileType.CONNECTIONS:
            return SumoConnectionsDefinitionsFile
        elif self == SumoIntermediateFileType.NODES:
            return SumoNodeDefinitionsFile
        elif self == SumoIntermediateFileType.EDGES:
            return SumoEdgeDefinitionsFile
        elif self == SumoIntermediateFileType.TLLOGICS:
            return SumoTrafficLightDefinitionsFile
        elif self == SumoIntermediateFileType.TYPES:
            return SumoEdgeTypeDefinitionsFile
        else:
            raise ValueError(f"Unknown file type: {self}")


class SumoIntermediateProject(AbstractSumoProject):
    """
    Interface to an intermediate (=before netconvert) SUMO project.

    An intermediate project can not yet be simulated but must be processed by netconvert first.
    Each `SumoIntermediateFileType` can only occur once in an intermediate project.
    """

    def __init__(
        self,
        project_name: str,
        project_path: Path,
        temp_dir: Optional[TemporaryDirectory] = None,
    ) -> None:
        super().__init__(project_name, project_path, temp_dir)

        # Remember which files are part of the project.
        self._intermediate_file_names: dict[SumoIntermediateFileType, str] = {}
        self._intermediate_files: dict[SumoIntermediateFileType, SumoXmlFile] = {}

    @override
    def write(self):
        for file_type, file in self._intermediate_files.items():
            file_path = self.get_file_path(file_type)
            file.write_to_file(file_path)

    def get_file_name(
        self, sumo_intermediate_file_type: SumoIntermediateFileType
    ) -> str:
        if sumo_intermediate_file_type in self._intermediate_file_names:
            return self._intermediate_file_names[sumo_intermediate_file_type]

        return f"{self._project_name}{sumo_intermediate_file_type.get_file_extension()}"

    def get_file_path(
        self, sumo_intermediate_file_type: SumoIntermediateFileType
    ) -> Path:
        return self._project_path.joinpath(
            self.get_file_name(sumo_intermediate_file_type)
        )

    def create_file(
        self, sumo_intermediate_file_type: SumoIntermediateFileType
    ) -> SumoXmlFile:
        xml_file_type = sumo_intermediate_file_type.get_xml_file_type()

        xml_file = xml_file_type()  # type: ignore
        self._intermediate_file_names[sumo_intermediate_file_type] = self.get_file_name(
            sumo_intermediate_file_type
        )
        self._intermediate_files[sumo_intermediate_file_type] = xml_file

        return xml_file

    def cleanup(self) -> None:
        for file_type, _ in self._intermediate_files.items():
            file_path = self.get_file_path(file_type)
            file_path.unlink()

        self._intermediate_file_names = {}
        self._intermediate_files = {}


class SumoFileType(Enum):
    CONFIG = auto()
    NET = auto()
    ADDITIONAL = auto()
    VEHICLE_ROUTES = auto()
    VEHICLE_TRIPS = auto()
    PEDESTRIAN_ROUTES = auto()
    PEDESTRIAN_TRIPS = auto()

    def file_extension(self) -> str:
        if self == SumoFileType.CONFIG:
            return ".sumo.cfg"
        elif self == SumoFileType.NET:
            return ".net.xml"
        elif self == SumoFileType.ADDITIONAL:
            return ".add.xml"
        elif self == SumoFileType.VEHICLE_ROUTES:
            return ".vehicles.rou.xml"
        elif self == SumoFileType.VEHICLE_TRIPS:
            return ".vehicles.trips.xml"
        elif self == SumoFileType.PEDESTRIAN_ROUTES:
            return ".pedestrian.rou.xml"
        elif self == SumoFileType.PEDESTRIAN_TRIPS:
            return ".pedestrian.trips.xml"
        else:
            raise ValueError(f"Unknown file type: {self}")

    def get_xml_file_type(self) -> type[SumoXmlFile]:
        if self == SumoFileType.CONFIG:
            return SumoConfigFile
        elif self == SumoFileType.NET:
            return SumoNetDefinitionsFile
        elif self == SumoFileType.ADDITIONAL:
            return SumoAdditionalDefinitionsFile
        elif self == SumoFileType.VEHICLE_ROUTES:
            return SumoRouteDefintionsFile
        elif self == SumoFileType.PEDESTRIAN_ROUTES:
            return SumoRouteDefintionsFile
        else:
            raise ValueError(f"SUMO file type can not be loaded or written: {self}")


class SumoProject(AbstractSumoProject):
    """
    The SumoProject is a wrapper around the file structure, that is usually present in any SUMO project.
    """

    def __init__(
        self,
        project_name: str,
        project_path: Path,
        temp_dir: Optional[TemporaryDirectory] = None,
    ) -> None:
        super().__init__(project_name, project_path, temp_dir)

        self._file_names: dict[SumoFileType, str] = {}
        self._initialize_from_sumo_cfg()
        self._files: dict[SumoFileType, SumoXmlFile] = {}

    @classmethod
    def from_intermediate_sumo_project(
        cls, sumo_intermediate_project: SumoIntermediateProject
    ) -> Self:
        return cls(
            sumo_intermediate_project.project_name,
            sumo_intermediate_project.project_path,
            sumo_intermediate_project._temp_dir,
        )

    @override
    def write(self) -> None:
        sumo_cfg_file = SumoConfigFile()
        for file_type, file in self._files.items():
            file_path = self.get_file_path(file_type)
            file.write_to_file(file_path)

        for file_type in SumoFileType:
            if self.has_file(file_type):
                file_name = self.get_file_name(file_type)
                if file_type == SumoFileType.NET:
                    sumo_cfg_file.net_file = file_name
                elif file_type == SumoFileType.ADDITIONAL:
                    sumo_cfg_file.add_additional_file(file_name)
                elif file_type == SumoFileType.VEHICLE_ROUTES:
                    sumo_cfg_file.add_route_file(file_name)

        sumo_cfg_file.write_to_file(self.get_file_path(SumoFileType.CONFIG))

    def get_file_name(self, sumo_file_type: SumoFileType) -> str:
        if sumo_file_type in self._file_names:
            return self._file_names[sumo_file_type]
        return f"{self._project_name}{sumo_file_type.file_extension()}"

    def get_file_path(self, sumo_file_type: SumoFileType) -> Path:
        file_name = self.get_file_name(sumo_file_type)
        return self._project_path.joinpath(file_name)

    def has_file(self, sumo_file_type: SumoFileType) -> bool:
        return self.get_file_path(sumo_file_type).exists()

    def load_file(self, sumo_file_type: SumoFileType) -> Optional[SumoXmlFile]:
        if sumo_file_type in self._files:
            return self._files[sumo_file_type]

        file_path = self.get_file_path(sumo_file_type)

        if not file_path.exists():
            return None

        xml_file_type = sumo_file_type.get_xml_file_type()
        xml_file = xml_file_type.read_from_file(file_path)
        self._files[sumo_file_type] = xml_file

        return xml_file

    def get_file(self, sumo_file_type: SumoFileType) -> Optional[SumoXmlFile]:
        return self._files.get(sumo_file_type)

    def create_file(self, sumo_file_type: SumoFileType) -> SumoXmlFile:
        xml_file_type = sumo_file_type.get_xml_file_type()
        xml_file = xml_file_type()  # type: ignore

        self._file_names[sumo_file_type] = self.get_file_name(sumo_file_type)
        self._files[sumo_file_type] = xml_file

        return xml_file

    def _initialize_from_sumo_cfg(self) -> None:
        if self.has_file(SumoFileType.CONFIG):
            sumo_cfg_file = SumoConfigFile.read_from_file(
                self.get_file_path(SumoFileType.CONFIG)
            )

            if sumo_cfg_file.net_file:
                self._file_names[SumoFileType.NET] = sumo_cfg_file.net_file
