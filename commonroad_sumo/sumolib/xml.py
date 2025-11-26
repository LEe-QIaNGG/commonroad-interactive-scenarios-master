from abc import ABC, abstractmethod
from pathlib import Path
from typing import List, Union
from xml.etree import cElementTree as ET

from typing_extensions import Self

import sumolib


class SumoXmlError(Exception): ...


class SumoXmlDeserializationError(Exception): ...


class SumoXmlSerializationError(Exception): ...


class SumoXmlSerializable(ABC):
    @abstractmethod
    def to_xml_element(self) -> ET.Element: ...

    def to_xml(self) -> str:
        return ET.tostring(self.to_xml_element(), encoding="unicode")

    def __repr__(self) -> str:
        return str(self)

    def __str__(self) -> str:
        return self.to_xml()


class SumoXmlDeserializable(ABC):
    @classmethod
    @abstractmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self: ...

    @classmethod
    def from_xml(cls, xml_str: str) -> Self:
        return cls.from_xml_element(ET.fromstring(xml_str))


# TODO: Make this a metaclass so that the root tag cen be provided as a class argument instead
class SumoXmlFile(SumoXmlSerializable, SumoXmlDeserializable):
    def __init__(self, root_tag: str):
        self._root_tag = root_tag

        self._general_nodes: List[SumoXmlSerializable] = []

    def add_node(self, node: SumoXmlSerializable) -> None:
        self._general_nodes.append(node)

    def to_xml_element(self) -> ET.Element:
        root = ET.Element(self._root_tag)

        for node in self._general_nodes:
            root.append(node.to_xml_element())
        return root

    def to_xml_tree(self) -> ET.ElementTree:
        return ET.ElementTree(self.to_xml_element())

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        raise NotImplementedError(
            f"Tried to create {cls} from xml element, but this is currently not supported!"
        )

    def write_to_file(self, file_path: Union[str, Path]) -> None:
        xml_root = self.to_xml_tree()
        ET.indent(xml_root, space="\t")

        with open(str(file_path), "w") as f:
            sumolib.writeXMLHeader(f, script="CommonRoad Scenario Designer")
            xml_root.write(f, encoding="unicode", xml_declaration=False)

    @classmethod
    def read_from_file(cls, file_path: Union[str, Path]) -> Self:
        with open(str(file_path)) as f:
            xml_content = f.read()

        return cls.from_xml(xml_content)
