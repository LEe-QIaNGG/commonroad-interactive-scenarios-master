"""
This file contains a content handler for parsing sumo network xml files.
It uses other classes from this module to represent the road network.
"""

import os
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass
from enum import Enum, unique
from typing import (
    Callable,
    Dict,
    Iterable,
    List,
    Literal,
    Optional,
    Sequence,
    Set,
    Tuple,
    TypeVar,
    Union,
)
from xml.etree import cElementTree as ET

import numpy as np
from typing_extensions import Self, override

import sumolib
from commonroad_sumo.sumolib.xml import (
    SumoXmlDeserializable,
    SumoXmlDeserializationError,
    SumoXmlFile,
    SumoXmlSerializable,
)


def set_allowed_changes(xml_node: ET.Element, obj: Union["Connection", "Lane"]):
    """Adds allowed lange change directions to etree xml node"""
    return
    if obj.change_left_allowed and len(obj.change_left_allowed) != len(VehicleType):
        xml_node.set("changeLeft", " ".join(la.value for la in obj.change_left_allowed))
    elif len(obj.change_left_allowed) == 0:
        xml_node.set("changeLeft", VehicleType.CUSTOM1.value)
    if obj.change_right_allowed and len(obj.change_right_allowed) != len(VehicleType):
        xml_node.set(
            "changeRight", " ".join(la.value for la in obj.change_right_allowed)
        )
    elif len(obj.change_right_allowed) == 0:
        xml_node.set("changeRight", VehicleType.CUSTOM1.value)


_K = TypeVar("_K")
_V = TypeVar("_V")
_VV = TypeVar("_VV")


def _get_default(
    d: Dict[_K, _V],
    key: _K,
    default: _VV = None,
    map: Callable[[_V], _VV] = lambda x: x,
) -> _VV:
    try:
        return map(d[key])
    except KeyError:
        return default


def _parse_sumo_array_into_numpy_array(sumo_array_str: str) -> np.ndarray:
    return np.array([float(f) for f in sumo_array_str.split(",")])


@dataclass
class NetLocation(SumoXmlDeserializable):
    net_offset: np.ndarray
    conv_boundary: np.ndarray
    orig_boundary: np.ndarray
    proj_parameter: str

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        return cls(
            net_offset=_parse_sumo_array_into_numpy_array(
                xml_element.attrib["netOffset"]
            ),
            conv_boundary=_parse_sumo_array_into_numpy_array(
                xml_element.attrib["convBoundary"]
            ),
            orig_boundary=_parse_sumo_array_into_numpy_array(
                xml_element.attrib["origBoundary"]
            ),
            proj_parameter=xml_element.attrib["projParameter"],
        )


class Net(SumoXmlDeserializable):
    """The whole sumo network."""

    def __init__(
        self,
        version: Optional[str] = None,
        junction_corner_detail: Optional[float] = None,
        junction_link_detail: Optional[float] = None,
        limit_turn_speed: Optional[float] = None,
        location: Optional[NetLocation] = None,
    ):
        self.version: str = version if version is not None else ""
        self.location = location
        self.junction_corner_detail = junction_corner_detail
        self.junction_link_detial = junction_link_detail
        self.limit_turn_speed = limit_turn_speed

        self.types: Dict[str, EdgeType] = {}
        self.junctions: Dict[int, Junction] = {}
        self.edges: Dict[int, Edge] = {}
        # from_edge -> from_lane -> to_edge -> to_lane -> Connection
        self.connections: Dict[int, Dict[int, Dict[int, Dict[int, Connection]]]] = (
            defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
        )
        # id -> program_id -> TLSProgram
        self.tlss: Dict[str, TLSProgram] = {}

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        return cls(
            version=_get_default(xml_element.attrib, "version", None),
            junction_corner_detail=_get_default(
                xml_element.attrib, "junctionCornerDetail", None, float
            ),
            junction_link_detail=_get_default(
                xml_element.attrib, "junctionLinkDetail", None, float
            ),
            limit_turn_speed=_get_default(
                xml_element.attrib, "limitTurnSpeed", None, float
            ),
        )


def sumo_net_from_xml(file: str) -> Net:
    """
    Given a SUMO .net.xml file this function returns the parsed
    representation of it.
    :param file: Path to .net.xml file
    :return: parsed Net
    """
    if not os.path.isfile(file):
        raise RuntimeError(f"Invalid file path: {file}")
    if not file.endswith(".net.xml"):
        raise RuntimeError(f"Invalid file type {file}, required *.net.xml")

    root = ET.parse(file).getroot()
    net = Net.from_xml_element(root)
    for elem in root.iter():
        if elem.tag == "location":
            net.location = NetLocation.from_xml_element(elem)
        elif elem.tag == "type":
            edge_type = EdgeType.from_xml_element(elem)
            net.types[edge_type.id] = edge_type
        elif elem.tag == "tlLogic":
            program = TLSProgram.from_xml_element(elem)
            assert program.id not in net.tlss
            net.tlss[program.id] = program

        elif elem.tag == "junction":
            junction = Junction.from_xml_element(elem)
            net.junctions[junction.id] = junction

    for elem in root:
        if elem.tag == "edge":
            edge = Edge.from_xml_element(elem)
            net.edges[edge.id] = edge
        elif elem.tag == "connection":
            from_edge = _get_default(elem.attrib, "from", map=lambda f: net.edges[f])
            to_edge = _get_default(elem.attrib, "to", map=lambda t: net.edges[t])
            c = Connection(
                from_edge=from_edge,
                to_edge=to_edge,
                from_lane=_get_default(
                    elem.attrib, "fromLane", map=lambda idx: from_edge.lanes[int(idx)]
                ),
                to_lane=_get_default(
                    elem.attrib, "toLane", map=lambda idx: to_edge.lanes[int(idx)]
                ),
                direction=_get_default(elem.attrib, "dir", map=ConnectionDirection),
                tls=_get_default(elem.attrib, "tl", map=lambda tls: net.tlss[tls]),
                tl_link=_get_default(elem.attrib, "linkIndex", map=int),
                state=_get_default(elem.attrib, "state"),
                via_lane_id=_get_default(
                    elem.attrib, "via", map=lambda via: via.split(" ")
                ),
                shape=_get_default(elem.attrib, "shape", map=from_shape_string),
                keep_clear=_get_default(
                    elem.attrib, "keepClear", map=lambda k: bool(int(k))
                ),
                cont_pos=_get_default(elem.attrib, "contPos"),
            )
            net.connections[c.from_edge.id][c.from_lane.id][c.to_edge.id][
                c.to_lane.id
            ] = c

    for junction in net.junctions.values():

        def replace_lanes(lane_ids):
            if not lane_ids:
                return None
            lanes = []
            for lane_id in lane_ids:
                split = lane_id.split("_")
                lanes.append(net.edges["_".join(split[:-1])].lanes[int(split[-1])])
            return lanes

        junction.int_lanes = replace_lanes(junction.int_lanes)
        junction.inc_lanes = replace_lanes(junction.inc_lanes)
    return net


#
# Node
#
@unique
class NodeType(Enum):
    """
    Node types:
    If you leave out the type of the node, it is automatically guessed by netconvert but may not be the one you
    intended.
    The following types are possible, any other string is counted as an error and will yield in a program stop:
    taken from https://sumo.dlr.de/docs/Networks/PlainXML.html#connections_after_joining_nodes
    """

    # priority: Vehicles on a low-priority edge have to wait until vehicles on a high-priority edge
    # have passed the junction.
    PRIORITY = "priority"
    # traffic_light: The junction is controlled by a traffic light (priority rules are used to avoid collisions
    # if conflicting links have green light at the same time).
    TRAFFIC_LIGHT = "traffic_light"
    # traffic_light_unregulated: The junction is controlled by a traffic light without any further rules.
    # This may cause collision if unsafe signal plans are used.
    # Note, that collisions within the intersection will never be detected.
    TRAFFIC_LIGHT_UNREGULATED = "traffic_light_unregulated"
    # traffic_light_right_on_red: The junction is controlled by a traffic light as for type traffic_light.
    # Additionally, right-turning vehicles may drive in any phase whenever it is safe to do so (after stopping once).
    # This behavior is known as right-turn-on-red.
    TRAFFIC_LIGHT_RIGHT_ON_RED = "traffic_light_right_on_red"
    # right_before_left: Vehicles will let vehicles coming from their right side pass.
    RIGHT_BEFORE_LEFT = "right_before_left"
    # unregulated: The junction is completely unregulated - all vehicles may pass without braking;
    # Collision detection on the intersection is disabled but collisions beyond the intersection will
    # detected and are likely to occur.
    UNREGULATED = "unregulated"
    # priority_stop: This works like a priority-junction but vehicles on minor links always have to stop before passing
    PRIORITY_STOP = "priority_stop"
    # allway_stop: This junction works like an All-way stop
    ALLWAY_STOP = "allway_stop"
    # rail_signal: This junction is controlled by a rail signal. This type of junction/control is only useful for rails.
    RAIL_SIGNAL = "rail_signal"
    # rail_crossing: This junction models a rail road crossing.
    # It will allow trains to pass unimpeded and will restrict vehicles via traffic signals when a train is approaching.
    RAIL_CROSSING = "rail_crossing"
    # zipper: This junction connects edges where the number of lanes decreases and traffic needs
    # to merge zipper-style (late merging).
    ZIPPER = "zipper"


@unique
class RightOfWay(Enum):
    # Taken from: https://sumo.dlr.de/docs/Networks/PlainXML.html#right-of-way
    # This mode is useful if the priority attribute of the edges cannot be relied
    # on to determine right-of-way all by itself.
    # It sorts edges according to priority, speed and laneNumber. The 2 incoming edges with the highest position
    # are determined and will receive right-of-way. All other edges will be classified as minor.
    DEFAULT = "default"
    # This mode is useful for customizing right-of-way by tuning edge priority attributes.
    # The relationship between streams of different incoming-edge priority will be solely determined by edge priority.
    # For equal-priority values, turning directions are also evaluated.
    EDGE_PRIORITY = "edgePriority"


class Node(SumoXmlSerializable, SumoXmlDeserializable):
    """Nodes from a sumo network"""

    def __init__(
        self,
        id: int,
        node_type: NodeType,
        coord: np.ndarray,
        shape: np.ndarray = None,
        inc_lanes: List["Lane"] = None,
        int_lanes: List["Lane"] = None,
        tl: "TLSProgram" = None,
        right_of_way=RightOfWay.DEFAULT,
    ):
        self.id = id
        self.type = node_type
        self.coord = coord
        self._incoming: List[Edge] = []
        self._outgoing: List[Edge] = []
        self._foes: Dict[int, Edge] = {}
        self._prohibits: Dict[int, Edge] = {}
        self.inc_lanes: List[Lane] = inc_lanes if inc_lanes is not None else []
        self.int_lanes: List[Lane] = int_lanes if int_lanes is not None else []
        self.shape: Optional[np.ndarray] = shape
        self.tl = tl
        self.right_of_way = right_of_way
        self.zipper = True
        self.keep_clear = True

    def add_outgoing(self, edge: "Edge"):
        self._outgoing.append(edge)

    @property
    def outgoing(self) -> List["Edge"]:
        return self._outgoing

    def add_incoming(self, edge: "Edge"):
        self._incoming.append(edge)

    @property
    def incoming(self) -> List["Edge"]:
        return self._incoming

    def to_xml_element(self) -> ET.Element:
        """
        Converts this node to it's xml representation
        """
        node = ET.Element("node")
        node.set("id", str(self.id))
        node.set("type", str(self.type.value))
        for k, v in zip(["x", "y", "z"][: self.coord.shape[0]], self.coord):
            node.set(k, str(v))
        if self.incoming:
            node.set("incoming", " ".join([str(i.id) for i in self.incoming]))
        if self.outgoing:
            node.set("outgoing", " ".join([str(o.id) for o in self.outgoing]))
        if self._foes is not None:
            # TODO: convert foes
            pass
        if self._prohibits is not None:
            # TODO: convert prohibits
            pass
        if self.keep_clear is False:
            node.set("keepClear", "false")
        if self.inc_lanes:
            node.set("incLanes", " ".join([str(la.id) for la in self.inc_lanes]))
        if self.int_lanes:
            node.set("intLanes", " ".join([str(la.id) for la in self.int_lanes]))
        if self.shape is not None:
            node.set("shape", to_shape_string(self.shape))
        if self.tl is not None:
            node.set("tl", self.tl.id)
        node.set("rightOfWay", str(self.right_of_way.value))
        return node

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        x = _get_default(xml_element.attrib, "x", None, float)
        y = _get_default(xml_element.attrib, "y", None, float)
        z = _get_default(xml_element.attrib, "z", None, float)
        return cls(
            id=int(xml_element.attrib["int"]),
            node_type=NodeType(xml_element.attrib["type"]),
            coord=np.array([x, y, z] if z is not None else [x, y]),
            shape=_get_default(xml_element.attrib, "shape", None, from_shape_string),
            inc_lanes=_get_default(
                xml_element.attrib,
                "incLanes",
                None,
                lambda inc_lanes: inc_lanes.split(" ") if inc_lanes else None,
            ),
            int_lanes=_get_default(
                xml_element.attrib,
                "intLanes",
                None,
                lambda int_lanes: int_lanes.split(" ") if int_lanes else None,
            ),
        )

    def __str__(self):
        return "Node: " + str(self.id)

    def __hash__(self):
        return hash((self.id, self.type))

    def __eq__(self, other):
        return (
            self.id == other.id
            and self.type == other.type
            and self.tl == other.tl
            and self.right_of_way == other.right_of_way
        )

    def __ne__(self, other):
        return not self.__eq__(other)


#
# Junction
#


@unique
class JunctionType(Enum):
    DEAD_END = "dead_end"
    # the following is copied from NodeType, as Enum inheritance is not supported:

    PRIORITY = "priority"
    # traffic_light: The junction is controlled by a traffic light (priority rules are used to avoid collisions
    # if conflicting links have green light at the same time).
    TRAFFIC_LIGHT = "traffic_light"
    # traffic_light_unregulated: The junction is controlled by a traffic light without any further rules.
    # This may cause collision if unsafe signal plans are used.
    # Note, that collisions within the intersection will never be detected.
    TRAFFIC_LIGHT_UNREGULATED = "traffic_light_unregulated"
    # traffic_light_right_on_red: The junction is controlled by a traffic light as for type traffic_light.
    # Additionally, right-turning vehicles may drive in any phase whenever it is safe to do so (after stopping once).
    # This behavior is known as right-turn-on-red.
    TRAFFIC_LIGHT_RIGHT_ON_RED = "traffic_light_right_on_red"
    # right_before_left: Vehicles will let vehicles coming from their right side pass.
    RIGHT_BEFORE_LEFT = "right_before_left"
    # unregulated: The junction is completely unregulated - all vehicles may pass without braking;
    # Collision detection on the intersection is disabled but collisions beyond the intersection will
    # detected and are likely to occur.
    UNREGULATED = "unregulated"
    # priority_stop: This works like a priority-junction but vehicles on minor links always have to stop before passing
    PRIORITY_STOP = "priority_stop"
    # allway_stop: This junction works like an All-way stop
    ALLWAY_STOP = "allway_stop"
    # rail_signal: This junction is controlled by a rail signal. This type of junction/control is only useful for rails.
    RAIL_SIGNAL = "rail_signal"
    # rail_crossing: This junction models a rail road crossing.
    # It will allow trains to pass unimpeded and will restrict vehicles via traffic signals when a train is approaching.
    RAIL_CROSSING = "rail_crossing"
    # zipper: This junction connects edges where the number of lanes decreases and traffic needs
    # to merge zipper-style (late merging).
    ZIPPER = "zipper"


@dataclass
class JunctionRequest(SumoXmlSerializable, SumoXmlDeserializable):
    index: int
    response: List[bool]
    foes: List[bool]
    cont: int

    def to_xml_element(self) -> ET.Element:
        """Convert the JunctionRequest instance to an XML element."""
        request = ET.Element("request")
        request.set("index", str(self.index))
        request.set("response", "".join(["1" if bit else "0" for bit in self.response]))
        request.set("foes", "".join(["1" if bit else "0" for bit in self.foes]))
        request.set("cont", str(self.cont))
        return request

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> "JunctionRequest":
        """Create a JunctionRequest instance from an XML element."""
        return cls(
            index=_get_default(xml_element.attrib, "index", 0, int),
            response=_get_default(
                xml_element.attrib,
                "response",
                [],
                lambda response: [bool(int(bit)) for bit in response],
            ),
            foes=_get_default(
                xml_element.attrib,
                "foes",
                [],
                lambda foes: [bool(int(bit)) for bit in foes],
            ),
            cont=_get_default(xml_element.attrib, "cont", 0, int),
        )


class Junction(Node, SumoXmlSerializable):
    def __init__(
        self,
        id: int,
        junction_type: JunctionType,
        coord: np.ndarray,
        shape: np.ndarray = None,
        inc_lanes: List["Lane"] = None,
        int_lanes: List["Lane"] = None,
        requests: List[JunctionRequest] = [],
    ):
        super().__init__(id, junction_type, coord, shape, inc_lanes, int_lanes)
        self.id = id
        self.type = junction_type
        assert coord.shape == (2,) or coord.shape == (
            3,
        ), f"Coord has to have two or three values, was {coord}"
        self.coord = coord
        self.shape = shape
        self.inc_lanes = inc_lanes
        self.int_lanes = int_lanes
        self.requests: List[JunctionRequest] = requests

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        requests = []
        for request_xml_element in xml_element.iter("request"):
            requests.append(JunctionRequest.from_xml_element(request_xml_element))

        x = _get_default(xml_element.attrib, "x", None, float)
        y = _get_default(xml_element.attrib, "y", None, float)
        z = _get_default(xml_element.attrib, "z", None, float)
        return cls(
            id=int(xml_element.attrib["int"]),
            junction_type=JunctionType(xml_element.attrib["type"]),
            coord=np.array([x, y, z] if z is not None else [x, y]),
            shape=_get_default(xml_element.attrib, "shape", None, from_shape_string),
            inc_lanes=_get_default(
                xml_element.attrib,
                "incLanes",
                None,
                lambda inc_lanes: inc_lanes.split(" ") if inc_lanes else None,
            ),
            int_lanes=_get_default(
                xml_element.attrib,
                "intLanes",
                None,
                lambda int_lanes: int_lanes.split(" ") if int_lanes else None,
            ),
            requests=requests,
        )

    def __str__(self):
        return "Junction: " + str(self.id)


#
# Edge
#
@unique
class SpreadType(Enum):
    # From: https://sumo.dlr.de/docs/Networks/PlainXML.html#spreadtype
    # (default): The edge geometry is interpreted as the left side of the edge and lanes flare out to the right.
    # This works well if edges in opposite directions have the same (or rather reversed) geometry.
    RIGHT = "right"
    # The edge geometry is interpreted as the middle of the directional edge and lanes
    # flare out symmetrically to both sides.
    # This is appropriate for one-way edges
    CENTER = "center"
    # The edge geometry is interpreted as the middle of a bi-directional road.
    # This works well when both directional edges have a different lane number.
    ROAD_CENTER = "roadCenter"


class Edge(SumoXmlSerializable, SumoXmlDeserializable):
    """Edges from a sumo network"""

    def __init__(
        self,
        id: str,
        from_node: "Node",
        to_node: "Node",
        type_id: str = "",
        speed: float = None,
        priority: int = None,
        length: float = None,
        shape: np.ndarray = None,
        spread_type: SpreadType = SpreadType.RIGHT,
        allow: List["VehicleType"] = None,
        disallow: List["VehicleType"] = None,
        width: float = None,
        name: str = None,
        end_offset: float = None,
        sidewalk_width: float = None,
    ):
        self.id = id
        self.from_node = from_node
        self.to_node = to_node
        if from_node:
            from_node.add_outgoing(self)
        if to_node:
            to_node.add_incoming(self)
        self.type_id = type_id
        self._priority = priority
        self.speed = speed
        self.priority = priority
        self.length = length
        self.shape = shape
        self.spread_type = spread_type
        self.allow = allow
        self.disallow = disallow
        self.width = width
        self.name = name
        self.end_offset = end_offset
        self.sidewalk_width = sidewalk_width

        self._lanes: List["Lane"] = []
        self._incoming: Dict[Node, List[Edge]] = defaultdict(list)
        self._outgoing: Dict[Node, List[Edge]] = defaultdict(list)
        self._name = name

    @property
    def num_lanes(self) -> int:
        return len(self._lanes)

    @property
    def lanes(self) -> List["Lane"]:
        return self._lanes

    def add_lane(self, lane: "Lane") -> int:
        index = len(self._lanes)
        self._lanes.append(lane)
        self.speed = lane.speed
        self.length = lane.length
        return index

    def add_outgoing(self, edge: "Edge"):
        self._outgoing[edge.to_node].append(edge)

    def add_incoming(self, edge: "Edge"):
        self._incoming[edge.from_node].append(edge)

    @property
    def incoming(self) -> List["Edge"]:
        return [e for edges in self._incoming.values() for e in edges]

    @property
    def outgoing(self) -> List["Edge"]:
        return [e for edges in self._outgoing.values() for e in edges]

    # def getClosestLanePosDist(self, point, perpendicular=False):
    #     minDist = 1e400
    #     minIdx = None
    #     minPos = None
    #     for i, l in enumerate(self._lanes):
    #         pos, dist = l.getClosestLanePosAndDist(point, perpendicular)
    #         if dist < minDist:
    #             minDist = dist
    #             minIdx = i
    #             minPos = pos
    #     return minIdx, minPos, minDist

    # def rebuildShape(self):
    #     numLanes = len(self._lanes)
    #     if numLanes % 2 == 1:
    #         self._shape3D = self._lanes[int(numLanes / 2)].getShape3D()
    #     else:
    #         self._shape3D = []
    #         minLen = -1
    #         for l in self._lanes:
    #             if minLen == -1 or minLen > len(l.getShape()):
    #                 minLen = len(l.shape)
    #         for i in range(minLen):
    #             x = 0.
    #             y = 0.
    #             z = 0.
    #             for l in self._lanes:
    #                 x += l.getShape3D()[i][0]
    #                 y += l.getShape3D()[i][1]
    #                 z += l.getShape3D()[i][2]
    #             self._shape3D.append((x / float(numLanes), y / float(numLanes),
    #                                   z / float(numLanes)))
    #
    #     self._shapeWithJunctions3D = lane.add_junction_pos(self._shape3D,
    #                                                        self.from_node.getCoord3D(),
    #                                                        self.to_node.getCoord3D())
    #
    #     if self._rawShape3D == []:
    #         self._rawShape3D = [self.from_node.getCoord3D(), self.to_node.getCoord3D()]
    #
    #     # 2d - versions
    #     self._shape = [(x, y) for x, y, z in self._shape3D]
    #     self._shapeWithJunctions = [(x, y)
    #                                 for x, y, z in self._shapeWithJunctions3D]
    #     self._rawShape = [(x, y) for x, y, z in self._rawShape3D]

    # def setTLS(self, tls):
    #     self._tls = tls

    # def is_fringe(self, connections=None):
    #     """true if this edge has no incoming or no outgoing connections (except turnarounds)
    #        If connections is given, only those connections are considered"""
    #     if connections is None:
    #         return self.is_fringe(self._incoming) or self.is_fringe(
    #             self._outgoing)
    #     else:
    #         cons = sum([c for c in connections.values()], [])
    #         return len([
    #             c for c in cons if c._direction != Connection.LINKDIR_TURN
    #         ]) == 0
    #
    # def allows(self, vClass):
    #     """true if this edge has a lane which allows the given vehicle class"""
    #     for lane in self._lanes:
    #         if vClass in lane._allow:
    #             return True
    #     return False

    def to_xml_element(self) -> ET.Element:
        edge = ET.Element("edge")
        edge.set("id", str(self.id))
        edge.set("from", str(self.from_node.id))
        edge.set("to", str(self.to_node.id))
        if self.type_id:
            edge.set("type", str(self.type_id))
        if self.num_lanes > 0:
            edge.set("numLanes", str(self.num_lanes))
        if self.speed is not None:
            edge.set("speed", str(self.speed))
        if self.priority is not None:
            edge.set("priority", str(self.priority))
        if self.length is not None:
            edge.set("length", str(self.length))
        if self.shape is not None:
            edge.set("shape", to_shape_string(self.shape))
        edge.set("spreadType", str(self.spread_type.value))
        if self.allow:
            edge.set("allow", " ".join([str(a.value) for a in self.allow]))
        if self.disallow:
            edge.set("disallow", " ".join([str(a.value) for a in self.allow]))
        if self.width is not None:
            edge.set("width", str(self.width))
        if self.name is not None:
            edge.set("name", self.name)
        if self.end_offset is not None:
            edge.set("endOffset", str(self.end_offset))
        if self.sidewalk_width is not None:
            edge.set("sidewalkWidth", str(self.sidewalk_width))

        for lane in self._lanes:
            edge.append(lane.to_xml_element())
        return edge

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        lanes = []
        for lane_xml_element in xml_element.iter("lane"):
            lanes.append(Lane.from_xml_element(lane_xml_element))

        raise NotImplementedError()
        # TODO: implement
        # edge = cls(
        #     id=xml_element.attrib["id"],
        #     from_node=_get_default(
        #         xml_element.attrib, "from", map=lambda f: net.junctions[int(f)]
        #     ),
        #     to_node=_get_default(
        #         xml_element.attrib, "to", map=lambda f: net.junctions[int(f)]
        #     ),
        #     type_id=_get_default(xml_element.attrib, "type", ""),
        #     speed=_get_default(xml_element.attrib, "speed", None, float),
        #     priority=_get_default(xml_element.attrib, "priority", None, int),
        #     length=_get_default(xml_element.attrib, "length", None, float),
        #     shape=_get_default(xml_element.attrib, "shape", None, from_shape_string),
        #     spread_type=_get_default(
        #         xml_element.attrib, "spreadType", None, SpreadType
        #     ),
        # )

        # TODO: This cyclic referenc between lane and edge is ugly,
        # for lane in lanes:
        #     lane.edge = edge

        # return edge

    def __hash__(self):
        return hash(
            (self.id, self.from_node.id, self.to_node.id, self.type_id, *self._lanes)
        )

    def __eq__(self, other: "Edge"):
        return (
            isinstance(other, type(self))
            and self.id == other.id
            and self.from_node == other.from_node
            and self.to_node == other.to_node
            and self.type_id == other.type_id
            and len(self._lanes) == len(other._lanes)
            and all(x == y for x, y in zip(self._lanes, other._lanes))
        )

    def __ne__(self, other: "Edge"):
        return not self.__eq__(other)


#
# Lane
#


def add_junction_pos(shape, fromPos, toPos):
    """Extends shape with the given positions in case they differ from the
    existing endpoints. assumes that shape and positions have the same dimensionality"""
    result = list(shape)
    if fromPos != shape[0]:
        result = [fromPos] + result
    if toPos != shape[-1]:
        result.append(toPos)
    return result


class Lane(SumoXmlSerializable, SumoXmlDeserializable):
    """Lanes from a sumo network"""

    def __init__(
        self,
        edge: Edge,
        speed: float,
        length: float,
        width: float,
        allow: List["VehicleType"] = None,
        disallow: List["VehicleType"] = None,
        shape: np.ndarray = None,
    ):
        self._edge = edge
        self._speed = speed
        self._length = length
        self._width = width
        self._shape = shape if shape is not None else np.empty(0)
        self._shapeWithJunctions = None
        self._shapeWithJunctions3D = None
        self._outgoing: List["Connection"] = []
        self._adjacent_opposite = None  # added by Lisa
        self._allow: List["VehicleType"] = []
        self._disallow: List["VehicleType"] = []
        self._set_allow_disallow(allow, disallow)

        self._index = edge.add_lane(self)

    @property
    def id(self) -> str:
        return f"{self._edge.id}_{self.index}"

    def _set_allow_disallow(
        self,
        allow: Optional[List["VehicleType"]],
        disallow: Optional[List["VehicleType"]],
    ):
        if allow is not None and disallow is not None:
            assert set(allow).isdisjoint(set(disallow))
            self._allow = allow
            self._disallow = disallow
        elif allow:
            self._disallow: List["VehicleType"] = list(set(VehicleType) - set(allow))
        elif disallow:
            self._allow: List["VehicleType"] = list(set(VehicleType) - set(disallow))

    @property
    def edge(self) -> Edge:
        return self._edge

    @edge.setter
    def edge(self, edge: Edge):
        self._edge = edge

    @property
    def speed(self) -> float:
        return self._speed

    @speed.setter
    def speed(self, speed: float):
        self._speed = speed

    @property
    def length(self) -> float:
        return self._length

    @length.setter
    def length(self, length: float):
        self._length = length

    @property
    def width(self) -> float:
        return self._width

    @width.setter
    def width(self, width: float):
        self._width = width

    def setAdjacentOpposite(self, opposite_lane_id):
        self._adjacent_opposite = opposite_lane_id

    def getAdjacentOpposite(self):
        return self._adjacent_opposite

    @property
    def shape(self) -> np.ndarray:
        return self._shape

    @shape.setter
    def shape(self, shape: np.ndarray):
        self._shape = shape

    @property
    def bounding_box(self) -> Tuple[float, float, float, float]:
        s = self.shape
        xmin = float(np.min(s[:, 0]))
        xmax = float(np.max(s[:, 0]))
        ymin = float(np.min(s[:, 1]))
        ymax = float(np.max(s[:, 1]))
        assert xmin != xmax or ymin != ymax
        return xmin, ymin, xmax, ymax

    def getClosestLanePosAndDist(self, point, perpendicular=False):
        return sumolib.geomhelper.polygon.OffsetAndDistanceToPoint(
            point, self.getShape(), perpendicular
        )

    @property
    def index(self) -> int:
        return self._index

    @property
    def outgoing(self) -> List["Connection"]:
        return self._outgoing

    def add_outgoing(self, conn: "Connection"):
        self._outgoing.append(conn)

    @property
    def allow(self) -> List["VehicleType"]:
        return self._allow

    @allow.setter
    def allow(self, allow: List["VehicleType"]):
        self._set_allow_disallow(allow, None)

    @property
    def disallow(self) -> List["VehicleType"]:
        return self._disallow

    @disallow.setter
    def disallow(self, disallow: List["VehicleType"]):
        self._set_allow_disallow(None, disallow)

    def to_xml_element(self) -> ET.Element:
        """
        Converts this lane to it's xml representation
        """
        lane = ET.Element("lane")
        lane.set("index", str(self.index))
        if self.speed:
            lane.set("speed", str(self._speed))
        if self._length:
            lane.set("length", str(self._length))
        if len(self._shape) > 0:
            lane.set("shape", to_shape_string(self._shape))
        if self._width:
            lane.set("width", str(self._width))
        if self._allow:
            lane.set("allow", " ".join(a.value for a in self._allow))
        if self._disallow:
            lane.set("disallow", " ".join(d.value for d in self._disallow))
        return lane

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        """
        Creates a Lane instance from an XML element.
        """
        return cls(
            edge=None,
            speed=_get_default(xml_element.attrib, "speed", None, float),
            length=_get_default(xml_element.attrib, "length", None, float),
            width=_get_default(xml_element.attrib, "width", None, float),
            allow=_get_default(
                xml_element.attrib,
                "allow",
                None,
                lambda allow: [VehicleType(a) for a in allow.split(" ")],
            ),
            disallow=_get_default(
                xml_element.attrib,
                "disallow",
                None,
                lambda disallow: [VehicleType(a) for a in disallow.split(" ")],
            ),
            shape=_get_default(xml_element.attrib, "shape", None, from_shape_string),
        )

    def __hash__(self):
        return hash((self.edge.id, self.index))

    def __eq__(self, other: "Lane"):
        return (
            isinstance(other, type(self))
            and self.edge.id == other.edge.id
            and self.speed == other.speed
            and self.length == other.length
            and self.width == other.width
            and self._shapeWithJunctions == other._shapeWithJunctions
            and self._shapeWithJunctions3D == other._shapeWithJunctions3D
            and len(self.outgoing) == len(other.outgoing)
            and all(x == y for x, y in zip(self.outgoing, other.outgoing))
            and len(self.allow) == len(other.allow)
            and all(x == y for x, y in zip(self.allow, other.allow))
            and len(self.disallow) == len(other.disallow)
            and all(x == y for x, y in zip(self.disallow, other.disallow))
        )

    def __ne__(self, other: "Lane"):
        return not self.__eq__(other)


#
# Connection
#


def to_shape_string(shape: np.ndarray) -> str:
    """
    Convert a collection of points from format shape to string
    :param shape:
    :return: the same shape but in string format
    """
    return " ".join([",".join(str(p) for p in v) for v in shape])


def from_shape_string(shape: str) -> np.ndarray:
    """
    Convert a shape string to a ndarray
    :param shape:
    :return:
    """
    return np.asarray(
        [[float(c) for c in coords.split(",")] for coords in shape.split(" ")],
        dtype=float,
    )


@unique
class ConnectionDirection(Enum):
    # constants as defined in sumo/src/utils/xml/SUMOXMLDefinitions.cpp
    STRAIGHT = "s"
    TURN = "t"
    LEFT = "l"
    RIGHT = "r"
    PARTLEFT = "L"
    PARTRIGHT = "R"


class Connection(SumoXmlSerializable, SumoXmlDeserializable):
    """edge connection for a sumo network"""

    def __init__(
        self,
        from_edge: Edge,
        to_edge: Edge,
        from_lane: Lane,
        to_lane: Lane,
        direction: ConnectionDirection = None,
        tls: "TLSProgram" = None,
        tl_link: int = None,
        state=None,
        via_lane_id: List[str] = None,
        shape: Optional[np.ndarray] = None,
        keep_clear: bool = None,
        cont_pos=None,
        prohibits: List["Connection"] = [],
        change_left_allowed: Set["VehicleType"] = None,
        change_right_allowed: Set["VehicleType"] = None,
        forbidden=False,
    ):
        self._from = from_edge
        self._to = to_edge
        self._from_lane = from_lane
        self._to_lane = to_lane
        self._direction = direction
        self._tls = tls
        self._tl_link = tl_link
        self._state = state
        self._via: List[str] = via_lane_id if via_lane_id is not None else []
        self._shape = shape
        self._keep_clear = keep_clear
        self._cont_pos = cont_pos
        self._prohibits = prohibits
        self._change_left_allowed = None
        self._change_right_allowed = None
        self.change_left_allowed = change_left_allowed
        self.change_right_allowed = change_right_allowed
        self._forbidden = forbidden

    @property
    def from_edge(self) -> Edge:
        return self._from

    @from_edge.setter
    def from_edge(self, from_edge: Edge):
        self._from = from_edge

    @property
    def to_edge(self) -> Edge:
        return self._to

    @to_edge.setter
    def to_edge(self, to_edge: Edge):
        self._to = to_edge

    @property
    def from_lane(self) -> Lane:
        return self._from_lane

    @from_lane.setter
    def from_lane(self, from_lane: Lane):
        self._from_lane = from_lane

    @property
    def to_lane(self) -> Lane:
        return self._to_lane

    @to_lane.setter
    def to_lane(self, to_lane: Lane):
        self._to_lane = to_lane

    @property
    def via(self) -> Optional[List[str]]:
        return self._via

    @via.setter
    def via(self, via: List[str]):
        self._via = via

    @property
    def direction(self):
        return self._direction

    @property
    def tls(self):
        return self._tls

    @tls.setter
    def tls(self, tls: "TLSProgram"):
        self._tls = tls

    @property
    def tl_link(self) -> int:
        return self._tl_link

    @tl_link.setter
    def tl_link(self, tl_link: int):
        self._tl_link = tl_link

    def get_junction_index(self):
        return self._from.to_node.getLinkIndex(self)

    @property
    def junction(self) -> Node:
        return self._from.to_node

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state):
        self._state = state

    @property
    def shape(self) -> np.ndarray:
        return self._shape

    @shape.setter
    def shape(self, shape: np.ndarray):
        self._shape = shape

    @shape.setter
    def shape(self, shape: np.ndarray):
        self._shape = shape

    @property
    def prohibits(self) -> List["Connection"]:
        return self._prohibits

    @prohibits.setter
    def prohibits(self, prohibits):
        self._prohibits = prohibits if self._prohibits is not None else []

    @property
    def connection_string(self) -> str:
        return f"{self.from_lane.id}->{self.to_lane.id}"

    @property
    def change_left_forbidden(self) -> Set["VehicleType"]:
        return set(VehicleType) - self._change_left_allowed

    @change_left_forbidden.setter
    def change_left_forbidden(self, change_left_forbidden):
        self._change_left_allowed = (
            set(VehicleType) - set(change_left_forbidden)
            if change_left_forbidden is not None
            else set(VehicleType)
        )

    @property
    def change_right_forbidden(self) -> Set["VehicleType"]:
        return (
            set(VehicleType) - self._change_right_allowed
            if self._change_right_allowed is not None
            else set(VehicleType)
        )

    @change_right_forbidden.setter
    def change_right_forbidden(self, change_right_forbidden):
        self._change_right_allowed = (
            set(VehicleType) - set(change_right_forbidden)
            if change_right_forbidden is not None
            else set(VehicleType)
        )

    @property
    def change_left_allowed(self) -> Set["VehicleType"]:
        return self._change_left_allowed

    @change_left_allowed.setter
    def change_left_allowed(self, change_left_allowed):
        self._change_left_allowed = (
            set(change_left_allowed)
            if change_left_allowed is not None
            else set(VehicleType)
        )

    @property
    def change_right_allowed(self) -> Set["VehicleType"]:
        return self._change_right_allowed

    @change_right_allowed.setter
    def change_right_allowed(self, change_right_allowed):
        self._change_right_allowed = (
            set(change_right_allowed)
            if change_right_allowed is not None
            else set(VehicleType)
        )

    def to_xml_element(self) -> ET.Element:
        c = ET.Element("connection")
        c.set("from", str(self._from.id))
        c.set("to", str(self._to.id))
        c.set("fromLane", str(self._from_lane.index))
        c.set("toLane", str(self._to_lane.index))
        if self._via is not None:
            c.set("via", " ".join(self._via))
        if self._direction is not None:
            c.set("dir", str(self._direction))
        if self._tls is not None:
            c.set("tl", str(self._tls.id))
        if self._tl_link is not None:
            c.set("linkIndex", str(self._tl_link))
        if self._state is not None:
            c.set("state", str(self._state))
        if self._forbidden is True:
            c.set("disallow", "all")
        if self._shape is not None:
            c.set("shape", to_shape_string(self._shape))
        if self._keep_clear is not None:
            c.set("keepClear", "true" if self._keep_clear is True else "false")
        if self._cont_pos is not None:
            c.set("contPos", str(self._cont_pos))

        for prohibit in self.prohibits:
            x = ET.Element("prohibition")
            x.set("prohibitor", self.connection_string)
            x.set("prohibited", prohibit.connection_string)
            c.append(x)

        set_allowed_changes(c, self)
        return c

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        raise NotImplementedError()
        # TODO: implement
        # return cls(
        #     from_edge=from_edge,
        #     to_edge=to_edge,
        #     from_lane=_get_default(
        #         xml_element.attrib,
        #         "fromLane",
        #         map=lambda idx: from_edge.lanes[int(idx)],
        #     ),
        #     to_lane=_get_default(
        #         xml_element.attrib, "toLane", map=lambda idx: to_edge.lanes[int(idx)]
        #     ),
        #     direction=_get_default(xml_element.attrib, "dir", map=ConnectionDirection),
        #     tls=_get_default(xml_element.attrib, "tl", map=lambda tls: net.tlss[tls]),
        #     tl_link=_get_default(xml_element.attrib, "linkIndex", map=int),
        #     state=_get_default(xml_element.attrib, "state"),
        #     via_lane_id=_get_default(
        #         xml_element.attrib, "via", map=lambda via: via.split(" ")
        #     ),
        #     shape=_get_default(xml_element.attrib, "shape", map=from_shape_string),
        #     keep_clear=_get_default(
        #         xml_element.attrib, "keepClear", map=lambda k: bool(int(k))
        #     ),
        #     cont_pos=_get_default(xml_element.attrib, "contPos"),
        # )

    def __hash__(self):
        return hash(
            (
                self._from.id,
                self._to.id,
                self._from_lane.id,
                self._to_lane.id,
                self._direction,
                self._tls,
                self._tl_link,
                self._state,
                len(self._via) if self._via else 0,
                self._keep_clear,
                self._cont_pos,
            )
        )

    def __eq__(self, other: "Connection"):
        return (
            isinstance(self, other)
            and self._from == other._from
            and self._to == other._to
            and self._direction == other._direction
            and self._tls == other._tls
            and self._tl_link == other._tl_link
            and self._state == other._state
            and len(self._via) == len(other._via)
            and all(x == y for x, y in zip(self._via, other._via))
            and self._keep_clear == other._keep_clear
            and self._cont_pos == other._cont_pos
        )

    def __ne__(self, other: "Connection"):
        return not self.__eq__(other)


#
# Crossings
#


class Crossing(SumoXmlSerializable):
    def __init__(
        self,
        node: Node,
        edges: Iterable[Edge],
        priority: bool = None,
        width: float = None,
        shape=None,
        link_index: int = None,
        link_index_2: int = None,
        discard: bool = None,
    ):
        self.node = node
        self.edges = edges
        self.priority = priority
        self.width = width
        self.shape = shape
        self.link_index = link_index
        self.link_index_2 = link_index_2
        self.discard = discard

    def __str__(self) -> str:
        return str(self.to_xml())

    def to_xml_element(self) -> ET.Element:
        c = ET.Element("crossing")
        c.set("node", str(self.node.id))
        c.set("edges", " ".join(str(edge.id) for edge in self.edges))
        if self.priority is not None:
            c.set("priority", str(self.priority))
        if self.width is not None:
            c.set("width", str(self.width))
        if self.shape is not None:
            c.set(
                "shape",
                " ".join([",".join(str(coord) for coord in v) for v in self.shape]),
            )
        if self.link_index is not None:
            c.set("linkIndex", str(self.link_index))
        if self.link_index_2 is not None:
            c.set("linkIndex2", str(self.link_index_2))
        if self.discard is not None:
            c.set("discard", str(self.discard))
        return c


#
# Edge Type Manager
#
def _bool_to_str(b: bool) -> str:
    return "true" if b else "false"


def _str_to_bool(s: str) -> bool:
    return s == "true"


class EdgeType(SumoXmlSerializable, SumoXmlDeserializable):
    def __init__(
        self,
        id: str,
        allow: List["VehicleType"] = None,
        disallow: List["VehicleType"] = None,
        discard: bool = False,
        num_lanes: int = -1,
        oneway=False,
        priority: int = 0,
        speed: float = 13.89,
        sidewalk_width: float = -1,
    ):
        """
        Constructs a SUMO Edge Type
        Documentation from: https://sumo.dlr.de/docs/SUMO_edge_type_file.html
        :param id: The name of the road type. This is the only mandatory attribute.
        For OpenStreetMap data, the name could, for example, be highway.trunk or highway.residential.
        For ArcView data, the name of the road type is a number.
        :param allow: List of allowed vehicle classes
        :param disallow: List of not allowed vehicle classes
        :param discard: If "yes", edges of that type are not imported. This parameter is optional and defaults to false.
        :param num_lanes: The number of lanes on an edge. This is the default number of lanes per direction.
        :param oneway: If "yes", only the edge for one direction is created during the import.
        (This attribute makes no sense for SUMO XML descriptions but, for example, for OpenStreetMap files.)
        :param priority: A number, which determines the priority between different road types.
        netconvert derives the right-of-way rules at junctions from the priority.
        The number starts with one; higher numbers represent more important roads.
        :param speed: The default (implicit) speed limit in m/s.
        :param sidewalk_width: The default width for added sidewalks (defaults to -1 which disables extra sidewalks).
        """
        self.id = id
        assert not (
            allow and disallow and set(allow) & set(disallow)
        ), f"allow and disallow contain common elements {set(allow) & set(disallow)}"
        self.allow: List["VehicleType"] = []
        if allow:
            assert set(allow).issubset(
                set(VehicleType)
            ), f"allow contains invalid classes {set(allow) - set(VehicleType)}"
            self.allow: List["VehicleType"] = allow

        self.disallow: List["VehicleType"] = []
        if disallow:
            assert set(disallow).issubset(
                set(VehicleType)
            ), f"disallow contains invalid classes {set(disallow) - set(VehicleType)}"
            self.disallow: List["VehicleType"] = disallow

        self.discard = discard
        self.num_lanes = num_lanes
        self.oneway = oneway
        self.priority = priority
        self.speed = speed
        self.sidewalk_width = sidewalk_width

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        """
        Creates an instance of this class from the given xml representation
        :param xml:
        :return:
        """

        def str_to_vehicle_type(value: str) -> VehicleType:
            return VehicleType(value)

        return cls(
            id=xml_element.attrib["id"],
            allow=_get_default(
                xml_element.attrib,
                "allow",
                [],
                lambda sp: [VehicleType(s) for s in sp.split(" ")],
            ),
            disallow=_get_default(
                xml_element.attrib,
                "disallow",
                [],
                lambda sp: [str_to_vehicle_type(s) for s in sp.split(" ")],
            ),
            discard=_get_default(xml_element.attrib, "discard", False, _str_to_bool),
            num_lanes=_get_default(xml_element.attrib, "numLanes", -1, int),
            oneway=_get_default(xml_element.attrib, "oneway", False, _str_to_bool),
            priority=_get_default(xml_element.attrib, "priority", 0, int),
            speed=_get_default(xml_element.attrib, "speed", 13.89, float),
            sidewalk_width=_get_default(xml_element.attrib, "sidewalkWidth", -1, int),
        )

    def to_xml_element(self) -> ET.Element:
        """
        Converts this node to it's xml representation
        :return: xml representation of this EdgeType
        """
        node = ET.Element("type")
        node.set("id", str(self.id))
        if self.allow:
            node.set("allow", " ".join(a.value for a in self.allow))
        if self.disallow:
            node.set("disallow", " ".join(d.value for d in self.disallow))
        if self.discard:
            node.set("discard", _bool_to_str(self.discard))
        if self.num_lanes != -1:
            node.set("numLanes", str(self.num_lanes))
        if self.oneway:
            node.set("oneway", _bool_to_str(self.oneway))
        if self.priority:
            node.set("priority", str(self.priority))
        if self.speed:
            node.set("speed", f"{self.speed:.2f}")
        if self.sidewalk_width > 0:
            node.set("sidewalkWidth", f"{self.sidewalk_width:.2f}")

        return node


class EdgeTypes(SumoXmlFile):
    def __init__(self, types: Dict[str, EdgeType] = None):
        self.types: Dict[str, EdgeType] = types if types else dict()

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        types: Dict[str, EdgeType] = {}
        for edge_type in xml_element.iter("type"):
            types[edge_type.get("id")] = EdgeType.from_xml_element(edge_type)
        return cls(types)

    def to_xml_element(self) -> ET.Element:
        types = ET.Element("types")
        types.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
        types.set(
            "xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/types_file.xsd"
        )
        for type_id, type in self.types.items():
            types.append(type.to_xml_element())
        return types

    def _create_from_update(
        self, old_id: str, attr: str, value: any
    ) -> Optional[EdgeType]:
        if old_id not in self.types:
            return None
        edge_type = self.types[old_id]

        val_rep = str(value)
        if isinstance(value, Iterable):
            val_rep = "_".join([str(v) for v in value])

        new_id = f"{edge_type.id}_{attr}_{val_rep}"
        if new_id in self.types:
            return self.types[new_id]

        new_type = deepcopy(edge_type)
        new_type.id = new_id
        setattr(new_type, attr, value)
        self.types[new_type.id] = new_type
        return new_type

    def create_from_update_priority(
        self, old_id: str, priority: int
    ) -> Optional[EdgeType]:
        return self._create_from_update(old_id, "priority", priority)

    def create_from_update_speed(self, old_id: str, speed: float) -> Optional[EdgeType]:
        return self._create_from_update(old_id, "speed", round(speed, 2))

    def create_from_update_oneway(
        self, old_id: str, oneway: bool
    ) -> Optional[EdgeType]:
        return self._create_from_update(old_id, "oneway", oneway)

    def create_from_update_allow(
        self, old_id: str, allow: List["VehicleType"]
    ) -> Optional[EdgeType]:
        new_type = self._create_from_update(old_id, "allow", allow)
        # setattr(new_type, "disallow", list(set(new_type.disallow) - set(new_type.allow)))
        return new_type

    def create_from_update_disallow(
        self, old_id: str, disallow: List["VehicleType"]
    ) -> Optional[EdgeType]:
        new_type = self._create_from_update(old_id, "disallow", disallow)
        # setattr(new_type, "allow", list(set(new_type.allow) - set(new_type.disallow)))
        return new_type


#
# Traffic Light Systems
#
class SignalState(Enum):
    """
    Adapted from: https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#tllogic62_attributes
    """

    # 'red light' for a signal - vehicles must stop
    RED = "r"
    # 'amber (yellow) light' for a signal -
    # vehicles will start to decelerate if far away from the junction, otherwise they pass
    YELLOW = "y"
    # 'green light' for a signal, no priority -
    # vehicles may pass the junction if no vehicle uses a higher priorised foe stream,
    # otherwise they decelerate for letting it pass.
    # They always decelerate on approach until they are within the configured visibility distance
    GREEN = "g"
    # 'green light' for a signal, priority -
    # vehicles may pass the junction
    GREEN_PRIORITY = "G"
    # 'green right-turn arrow' requires stopping -
    # vehicles may pass the junction if no vehicle uses a higher priorised foe stream.
    # They always stop before passing.
    # This is only generated for junction type traffic_light_right_on_red.
    GREEN_TURN_RIGHT = "s"
    # 'red+yellow light' for a signal, may be used to indicate upcoming
    # green phase but vehicles may not drive yet (shown as orange in the gui)
    RED_YELLOW = "u"
    # 'off - blinking' signal is switched off, blinking light indicates vehicles have to yield
    BLINKING = "o"
    # 'off - no signal' signal is switched off, vehicles have the right of way
    NO_SIGNAL = "O"


class Phase(SumoXmlSerializable, SumoXmlDeserializable):
    def __init__(
        self,
        duration: float,
        state: List[SignalState],
        min_dur: int = None,
        max_dur: int = None,
        name: str = None,
        next: List[int] = None,
    ):
        """
        Adapted from: https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#tllogic62_attributes
        :param duration: The duration of the phase (sec)
        :param state: The traffic light states for this phase, see below
        :param min_dur: The minimum duration of the phase when using type actuated. Optional, defaults to duration.
        :param max_dur: The maximum duration of the phase when using type actuated. Optional, defaults to duration.
        :param name: An optional description for the phase. This can be used to establish the
        correspondence between SUMO-phase-indexing and traffic engineering phase names.
        :param next: The next phase in the cycle after the current.
        This is useful when adding extra transition phases to a traffic light plan which are not part of every cycle.
        Traffic lights of type 'actuated' can make use of a list of indices for
        selecting among alternative successor phases.
        """
        self.duration = duration
        self.state = state
        self.min_dur = min_dur
        self.max_dur = max_dur
        self.name = name
        self.next = next

    def to_xml_element(self) -> ET.Element:
        phase = ET.Element("phase")
        phase.set("duration", str(self.duration))
        phase.set("state", "".join([s.value for s in self.state]))
        if self.min_dur is not None:
            phase.set("minDur", str(self.min_dur))
        if self.max_dur is not None:
            phase.set("maxDur", str(self.max_dur))
        if self.name is not None:
            phase.set("name", str(self.name))
        if self.next is not None:
            phase.set("next", str(self.next))
        return phase

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        return cls(
            duration=_get_default(xml_element.attrib, "duration", 0.0, float),
            state=_get_default(
                xml_element.attrib,
                "state",
                [],
                lambda state: [SignalState(s) for s in state],
            ),
            min_dur=_get_default(xml_element.attrib, "minDur", None, int),
            max_dur=_get_default(xml_element.attrib, "maxDur", None, int),
            name=_get_default(xml_element.attrib, "name"),
            next=_get_default(
                xml_element.attrib,
                "next",
                None,
                lambda n: [int(i) for i in n.split(" ")],
            ),
        )


class TLSType(Enum):
    """
    Adapted from: https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html
    The type of the traffic light
     - fixed phase durations,
     - phase prolongation based on time gaps between vehicles (actuated),
     - or on accumulated time loss of queued vehicles (delay_based)
    """

    STATIC = "static"
    ACTUATED = "actuated"
    DELAY_BASED = "delay_based"


class TLSProgram(SumoXmlSerializable, SumoXmlDeserializable):
    def __init__(
        self, id: str, offset: int, program_id: str, tls_type: TLSType = TLSType.STATIC
    ):
        """
        Adapted from: https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#tllogic62_attributes
        :param id: The id of the traffic light. This must be an existing traffic light id in the .net.xml file.
        Typically the id for a traffic light is identical with the junction id.
        The name may be obtained by right-clicking the red/green bars in front of a controlled intersection.
        :param offset: The initial time offset of the program
        :param program_id: The id of the traffic light program;
        This must be a new program name for the traffic light id.
        Please note that "off" is reserved, see below.
        :param tls_type: The type of the traffic light (fixed phase durations, phase prolongation based on time
        gaps between vehicles (actuated), or on accumulated time loss of queued vehicles (delay_based) )
        """
        self._id = id
        self._type = tls_type
        self._offset = offset
        self._program_id = program_id
        self._phases: List[Phase] = []

    @property
    def id(self) -> str:
        return self._id

    @property
    def program_id(self) -> str:
        return self._id

    @property
    def phases(self) -> List[Phase]:
        return self._phases

    @phases.setter
    def phases(self, phases: List[Phase]):
        self._phases = phases

    @property
    def offset(self) -> int:
        return self._offset

    @offset.setter
    def offset(self, offset: int):
        self._offset = offset

    def add_phase(self, phase: Phase):
        self._phases.append(phase)

    def to_xml_element(self) -> ET.Element:
        tl = ET.Element("tlLogic")
        tl.set("id", self._id)
        tl.set("type", str(self._type.value))
        tl.set("programID", str(self._program_id))
        tl.set("offset", str(int(self._offset)))
        for phase in self._phases:
            tl.append(ET.fromstring(phase.to_xml()))

        return tl

    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        phases = []
        for phase_xml_element in xml_element.iter("phase"):
            phases.append(Phase.from_xml_element(phase_xml_element))

        return cls(
            id=xml_element.attrib["id"],
            offset=_get_default(xml_element.attrib, "offset", 0, int),
            program_id=xml_element.attrib["programID"],
            tls_type=TLSType(xml_element.attrib["type"]),
        )


class TLS(SumoXmlFile):
    """Traffic Light Signal, managing TLSPrograms for SUMO"""

    def __init__(self):
        super().__init__(root_tag="tlLogics")
        self._connections: List[Connection] = []
        self._maxConnectionNo = -1
        self._programs: Dict[str, Dict[str, TLSProgram]] = defaultdict(dict)

    @property
    def connections(self) -> List[Connection]:
        return self._connections

    def add_connection(self, connection: Connection):
        self._connections.append(connection)

    @property
    def programs(self) -> Dict[str, Dict[str, TLSProgram]]:
        return self._programs

    def add_program(self, program: TLSProgram):
        self._programs[program._id][program._program_id] = program

    def clear_programs(self):
        self._programs.clear()

    def to_xml_tree(self) -> ET.ElementTree:
        tl = ET.Element("tlLogics")
        for programs in self._programs.values():
            for program in programs.values():
                tl.append(program.to_xml_element())
        for c in self._connections:
            tl.append(c.to_xml_element())
        return ET.ElementTree(tl)


#
# Roundabout
#


class Roundabout(SumoXmlSerializable):
    def __init__(self, edges: List[Edge] = None):
        self._edges = edges if edges is not None else []

    # @property
    # def nodes(self) -> List[Node]:
    #     return self._nodes
    #
    # @nodes.setter
    # def nodes(self, nodes: List[Node]):
    #     self._nodes = nodes

    @property
    def edges(self) -> List[Edge]:
        return self._edges

    @edges.setter
    def edges(self, edges: List[Edge]):
        self._edges = edges

    def to_xml_element(self) -> ET.Element:
        roundabout = ET.Element("roundabout")
        roundabout.set("edges", " ".join([str(e.id) for e in self.edges]))
        return roundabout


#
# Enums
#
@unique
class VehicleType(Enum):
    """taken from sumo/src/utils/common/SUMOVehicleClass.cpp
    "public_emergency",  # deprecated
    "public_authority",  # deprecated
    "public_army",       # deprecated
    "public_transport",  # deprecated
    "transport",         # deprecated
    "lightrail",         # deprecated
    "cityrail",          # deprecated
    "rail_slow",         # deprecated
    "rail_fast",         # deprecated
    """

    PRIVATE = "private"
    EMERGENCY = "emergency"
    AUTHORITY = "authority"
    ARMY = "army"
    VIP = "vip"
    PASSENGER = "passenger"
    HOV = "hov"
    TAXI = "taxi"
    BUS = "bus"
    COACH = "coach"
    DELIVERY = "delivery"
    TRUCK = "truck"
    TRAILER = "trailer"
    TRAM = "tram"
    RAIL_URBAN = "rail_urban"
    RAIL = "rail"
    RAIL_ELECTRIC = "rail_electric"
    MOTORCYCLE = "motorcycle"
    MOPED = "moped"
    BICYCLE = "bicycle"
    PEDESTRIAN = "pedestrian"
    EVEHICLE = "evehicle"
    SHIP = "ship"
    CUSTOM1 = "custom1"
    CUSTOM2 = "custom2"


SUMO_VEHICLE_CLASSES: Tuple[str] = tuple(str(vehicle.value) for vehicle in VehicleType)


class SumoFlow(SumoXmlSerializable):
    def __init__(
        self,
        # TODO: id is optional in SUMO spec
        id_: str,
        route: Optional[str] = None,
        start_edge: Optional[Union[Edge, str]] = None,
        end_edge: Optional[Union[Edge, str]] = None,
        start_lane: Optional[Union[Lane, str]] = None,
        end_lane: Optional[Union[Lane, str]] = None,
        vehicle_type: Optional[str] = None,
        period: Optional[float] = None,
        probability: Optional[float] = None,
        depart_speed: Optional[Union[float, Literal["max"], Literal["desired"]]] = None,
    ):
        self._id = id_
        self._route = route
        if route is None and (start_edge is None or end_edge is None):
            raise ValueError(
                "Failed to create SumoFlow: needs either 'route' or 'start_edge' and 'end_edge'."
            )
        self._start_edge = start_edge
        self._start_lane = start_lane
        self._end_edge = end_edge
        self._end_lane = end_lane
        self._vehicle_type = vehicle_type
        if period is not None and probability is not None:
            raise ValueError(
                f"Cannot create SUMO flow {id_}: 'period' and 'probability' are mutually exclusive!"
            )
        self._period = period
        if probability is not None and (probability < 0.0 or probability > 1.0):
            raise ValueError(
                f"Cannot create flow {id_}: probability must be in the interval [0,1] but is {probability}"
            )
        self._probability = probability
        self._depart_speed = depart_speed

    def to_xml_element(self) -> ET.Element:
        flow = ET.Element("flow")
        flow.set("id", self._id)

        if self._route is not None:
            flow.set("route", str(self._route))

        if self._start_edge is not None:
            if isinstance(self._start_edge, Edge):
                flow.set("from", str(self._start_edge.id))
            else:
                flow.set("from", str(self._start_edge))

        if self._end_edge is not None:
            if isinstance(self._end_edge, Edge):
                flow.set("to", str(self._end_edge.id))
            else:
                flow.set("to", str(self._end_edge))

        if self._start_lane is not None:
            if isinstance(self._start_lane, Lane):
                flow.set("departLane", str(self._start_lane.index))
            else:
                flow.set("departLane", str(self._start_lane))

        if self._end_lane is not None:
            if isinstance(self._end_lane, Lane):
                flow.set("arrivalLane", str(self._end_lane.index))
            else:
                flow.set("arrivalLane", str(self._end_lane))

        if self._vehicle_type is not None:
            flow.set("type", str(self._vehicle_type))

        if self._period is not None:
            flow.set("period", f"exp({self._period})")

        if self._probability is not None:
            flow.set("probability", str(self._probability))

        if self._depart_speed is not None:
            flow.set("departSpeed", str(self._depart_speed))

        flow.set("end", str(86400))

        return flow


@dataclass
class SumoVehicleType(SumoXmlSerializable):
    vehicle_type_id: str
    gui_shape: str
    vehicle_class: str
    probability: Optional[float] = None
    acceleration: Optional[float] = None
    decceleration: Optional[float] = None
    max_speed: Optional[float] = None
    length: Optional[float] = None
    width: Optional[float] = None
    min_gap: Optional[float] = None
    lc_strategic: Optional[float] = None
    lc_speed_gain: Optional[float] = None
    lc_cooperative: Optional[float] = None
    lc_sigma: Optional[float] = None
    lc_impatience: Optional[float] = None
    lc_keep_right: Optional[float] = None
    lc_max_speed_lat_standing: Optional[float] = None
    sigma: Optional[float] = None
    impatience: Optional[float] = None
    speed_dev: Optional[float] = None
    speed_factor: Optional[float] = None

    @classmethod
    def from_vehicle_type(cls, vehicle_type: VehicleType) -> "SumoVehicleType":
        return cls(
            vehicle_type_id=vehicle_type.value,
            gui_shape=vehicle_type.value,
            vehicle_class=vehicle_type.value,
        )

    def to_xml_element(self) -> ET.Element:
        v_type_node = ET.Element("vType")
        v_type_node.set("id", self.vehicle_type_id)
        v_type_node.set("guiShape", self.vehicle_type_id)
        v_type_node.set("vClass", self.vehicle_class)

        if self.probability is not None:
            v_type_node.set("probability", str(self.probability))

        if self.acceleration is not None:
            v_type_node.set("accel", str(self.acceleration))

        if self.decceleration is not None:
            v_type_node.set("deccel", str(self.decceleration))

        if self.max_speed is not None:
            v_type_node.set("maxSpeed", str(self.max_speed))

        if self.length is not None:
            v_type_node.set("length", str(self.length))

        if self.width is not None:
            v_type_node.set("width", str(self.width))

        if self.min_gap is not None:
            v_type_node.set("minGap", str(self.min_gap))

        if self.lc_strategic is not None:
            v_type_node.set("lcStrategic", str(self.lc_strategic))

        if self.lc_speed_gain is not None:
            v_type_node.set("lcSpeedGain", str(self.lc_speed_gain))

        if self.lc_cooperative is not None:
            v_type_node.set("lcCooperative", str(self.lc_cooperative))

        if self.lc_sigma is not None:
            v_type_node.set("lcSigma", str(self.lc_sigma))

        if self.lc_impatience is not None:
            v_type_node.set("lcImpatience", str(self.lc_impatience))

        if self.lc_keep_right is not None:
            v_type_node.set("lcKeepRight", str(self.lc_keep_right))

        if self.lc_max_speed_lat_standing is not None:
            v_type_node.set(
                "lcMaxSpeedLatStanding", str(self.lc_max_speed_lat_standing)
            )

        if self.sigma is not None:
            v_type_node.set("sigma", str(self.sigma))

        if self.impatience is not None:
            v_type_node.set("impatience", str(self.impatience))

        if self.speed_dev is not None:
            v_type_node.set("speedDev", str(self.speed_dev))

        if self.speed_factor is not None:
            v_type_node.set("speedFactor", str(self.speed_factor))

        return v_type_node


class SumoVehicleTypeDistribution(SumoXmlSerializable):
    def __init__(
        self,
        id_: str,
        v_types: Optional[Union[Sequence[str], Sequence[SumoVehicleType]]] = None,
        probabilities: Optional[Sequence[float]] = None,
    ) -> None:
        self._id = id_
        self._v_types = v_types
        self._probabilities = probabilities

    def to_xml_element(self) -> ET.Element:
        v_type_distribution_node = ET.Element("vTypeDistribution")
        v_type_distribution_node.set("id", self._id)

        if self._v_types is not None and len(self._v_types) > 0:
            if isinstance(self._v_types[0], SumoVehicleType):
                for v_type in self._v_types:
                    v_type_distribution_node.append(v_type.to_xml_element())
            else:
                v_type_distribution_node.set("vTypes", " ".join(self._v_types))

        if self._probabilities is not None:
            v_type_distribution_node.set(
                "probabilities",
                " ".join(str(probability) for probability in self._probabilities),
            )
        return v_type_distribution_node


class SumoRoute(SumoXmlSerializable):
    def __init__(
        self,
        edges: Sequence[str],
        id_: Optional[str] = None,
        probability: Optional[float] = None,
    ) -> None:
        self._id = id_
        self._edges = edges
        self._probability = probability

    def to_xml_element(self) -> ET.Element:
        route_node = ET.Element("route")
        if self._id is not None:
            route_node.set("id", self._id)

        route_node.set("edges", " ".join(self._edges))

        if self._probability is not None:
            route_node.set("probability", str(self._probability))

        return route_node


@dataclass
class SumoVehicle(SumoXmlSerializable):
    vehicle_id: str
    depart_time: float
    depart_speed: float
    depart_lane_id: int
    depart_pos: float
    arrival_lane_id: int
    arrival_pos: float
    vehicle_type: str
    edge_ids: Sequence[str]
    insertion_checks: bool = True

    def to_xml_element(self) -> ET.Element:
        vehicle_node = ET.Element("vehicle")
        vehicle_node.set("id", self.vehicle_id)
        vehicle_node.set("depart", str(self.depart_time))
        vehicle_node.set("departSpeed", str(self.depart_speed))
        vehicle_node.set("departLane", str(self.depart_lane_id))
        vehicle_node.set("departPos", str(self.depart_pos))
        vehicle_node.set("arrivalPos", str(self.arrival_pos))
        vehicle_node.set("arrivalLane", str(self.arrival_lane_id))
        vehicle_node.set("type", self.vehicle_type)

        if not self.insertion_checks:
            vehicle_node.set("insertionChecks", "none")

        route = SumoRoute(self.edge_ids)
        vehicle_node.append(route.to_xml_element())

        return vehicle_node


class SumoRouteDistribution(SumoXmlSerializable):
    def __init__(self, id_: str, routes: Iterable[str]) -> None:
        self._id = id_
        self._route_ids = routes

    def to_xml_element(self) -> ET.Element:
        route_distribution_node = ET.Element("routeDistribution")
        route_distribution_node.set("id", self._id)

        for route_id in self._route_ids:
            # TODO: use a seperate datatype for the refs?
            route_ref_node = ET.Element("route")
            route_ref_node.set("refId", route_id)
            route_distribution_node.append(route_ref_node)

        return route_distribution_node


class SumoTrip(SumoXmlSerializable):
    def __init__(self, id_: str, depart: str, arrival: str):
        self._id = id_
        self._depart = depart
        self._arrival = arrival

    def to_xml_element(self) -> ET.Element:
        trip_node = ET.Element("trip")
        trip_node.set("id", self._id)
        trip_node.set("depart", str(0))

        trip_node.set("from", self._depart)
        trip_node.set("to", self._arrival)

        return trip_node


class SumoConfigFile(SumoXmlFile):
    def __init__(
        self,
        net_file: Optional[str] = None,
        route_files: Optional[Set[str]] = None,
        additional_files: Optional[Set[str]] = None,
    ):
        super().__init__(root_tag="configuration")
        self._net_file = net_file
        self._route_files = route_files
        self._additional_files = additional_files

    @property
    def net_file(self) -> Optional[str]:
        return self._net_file

    @net_file.setter
    def net_file(self, value) -> None:
        self._net_file = value

    def get_net_file(self) -> Optional[str]:
        return self._net_file

    def add_route_file(self, route_file: str) -> None:
        if self._route_files is None:
            self._route_files = set()
        self._route_files.add(route_file)

    def add_additional_file(self, additional_file: str) -> None:
        if self._additional_files is None:
            self._additional_files = set()
        self._additional_files.add(additional_file)

    @override
    def to_xml_element(self) -> ET.Element:
        root = ET.Element(self._root_tag)

        input_element = ET.Element("input")

        if self._net_file is not None:
            net_file_element = ET.Element("net-file")
            net_file_element.set("value", self._net_file)
            input_element.append(net_file_element)

        if self._route_files is not None:
            route_files_element = ET.Element("route-files")
            route_files_element.set("value", ", ".join(self._route_files))
            input_element.append(route_files_element)

        if self._additional_files is not None:
            additional_files_element = ET.Element("additional-files")
            additional_files_element.set("value", ", ".join(self._additional_files))
            input_element.append(additional_files_element)

        root.append(input_element)
        return root

    @override
    @classmethod
    def from_xml_element(cls, xml_element: ET.Element) -> Self:
        if xml_element.tag != "configuration":
            raise SumoXmlDeserializationError(
                f"Failed to parse `SumoConfigFile` from XML element: root tag is '{xml_element.tag}' but must be 'configuration'!"
            )

        input_tag = xml_element.find("input")
        if input_tag is None:
            raise SumoXmlDeserializationError(
                "Failed to parse `SumoConfigFile` from XML element: 'input' tag is missing!"
            )

        net_file = None
        net_file_tag = input_tag.find("net-file")
        if net_file_tag is not None:
            possible_net_file = net_file_tag.attrib["value"]
            if len(possible_net_file) > 0:
                net_file = possible_net_file

        route_files = []
        route_files_tag = input_tag.find("route-files")
        if route_files_tag is not None:
            possible_route_files = route_files_tag.attrib["value"]
            if len(possible_route_files) > 0:
                route_files = possible_route_files.split(",")

        additional_files = []
        additional_files_tags = input_tag.find("additional-files")
        if additional_files_tags is not None:
            possible_additional_files = additional_files_tags.attrib["value"]
            if len(possible_additional_files) > 0:
                additional_files = possible_additional_files.split(",")

        return cls(net_file, set(route_files), set(additional_files))


class SumoNetDefinitionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="net")


class SumoRouteDefintionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="routes")


class SumoAdditionalDefinitionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="additional")


class SumoEdgeTypeDefinitionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="additional")


class SumoConnectionsDefinitionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="connections")


class SumoTrafficLightDefinitionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="tlLogics")


class SumoNodeDefinitionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="nodes")


class SumoEdgeDefinitionsFile(SumoXmlFile):
    def __init__(self):
        super().__init__(root_tag="edges")
