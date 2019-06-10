

import time
import json
import os
import tempfile
import threading
from collections import defaultdict, Iterable
import numpy as np
from lcm import LCM
from robotlocomotion import viewer2_comms_t
from director.thirdparty import transformations


class ClientIDFactory(object):
    def __init__(self):
        self.pid = os.getpid()
        self.counter = 0

    def new_client_id(self):
        self.counter += 1
        return "py_{:d}_{:d}".format(self.pid, self.counter)


CLIENT_ID_FACTORY = ClientIDFactory()


def to_lcm(data):
    msg = viewer2_comms_t()
    msg.utime = data["utime"]
    msg.format = "treeviewer_json"
    msg.format_version_major = 1
    msg.format_version_minor = 0
    msg.data = bytearray(json.dumps(data), encoding='utf-8')
    msg.num_bytes = len(msg.data)
    return msg


def serialize_transform(tform):
    return {
        "translation": list(transformations.translation_from_matrix(tform)),
        "quaternion": list(transformations.quaternion_from_matrix(tform))
    }


class GeometryData(object):
    __slots__ = ["geometry", "color", "transform"]
    def __init__(self, geometry, color=(1., 1., 1., 1.), transform=np.eye(4)):
        self.geometry = geometry
        self.color = color
        self.transform = transform

    def serialize(self):
        params = self.geometry.serialize()
        params["color"] = list(self.color)
        params["transform"] = serialize_transform(self.transform)
        return params


class BaseGeometry(object):
    def serialize(self):
        raise NotImplementedError()


class Box(BaseGeometry):
    __slots__ = ["lengths"]
    def __init__(self, lengths=[1,1,1]):
        self.lengths = lengths

    def serialize(self):
        return {
            "type": "box",
            "lengths": list(self.lengths)
        }


class Sphere(BaseGeometry):
    __slots__ = ["radius"]
    def __init__(self, radius=1):
        self.radius = radius

    def serialize(self):
        return {
            "type": "sphere",
            "radius": self.radius
        }


class Ellipsoid(BaseGeometry):
    __slots__ = ["radii"]
    def __init__(self, radii=[1,1,1]):
        self.radii = radii

    def serialize(self):
        return {
            "type": "ellipsoid",
            "radii": list(self.radii)
        }


class Cylinder(BaseGeometry):
    __slots__ = ["length", "radius"]
    def __init__(self, length=1, radius=1):
        self.length = length
        self.radius = radius

    def serialize(self):
        return {
            "type": "cylinder",
            "length": self.length,
            "radius": self.radius
        }


class Triad(BaseGeometry):
    __slots__ = ["tube", "scale"]
    def __init__(self, scale=1.0, tube=False):
        self.scale = scale
        self.tube = tube

    def serialize(self):
        return {
            "type": "triad",
            "scale": self.scale,
            "tube": self.tube
        }

class PointCloud(BaseGeometry):
    __slots__ = ["points", "channels"]
    def __init__(self, points, channels={}):
        self.points = points
        self.channels = channels

    def serialize(self):
        return {
            "type": "pointcloud",
            "points": [list(p) for p in self.points],
            "channels": {name: [list(c) for c in values] for (name, values) in self.channels.items()}
        }


class PolyLine(BaseGeometry):
    def __init__(self, points, radius=0.01, closed=False,
                 start_head=False, end_head=False,
                 head_radius=0.05, head_length=None):
        self.points = points
        self.radius = radius
        self.closed = closed
        self.start_head = start_head
        self.end_head = end_head
        self.head_radius = head_radius
        self.head_length = head_length if head_length is not None else head_radius

    def serialize(self):
        data = {
            "type": "line",
            "points": [list(p) for p in self.points],
            "radius": self.radius,
            "closed": self.closed
        }
        if self.start_head or self.end_head:
            data["start_head"] = self.start_head
            data["end_head"] = self.end_head
            data["head_radius"] = self.head_radius
            data["head_length"] = self.head_length
        return data


class LazyTree(object):
    __slots__ = ["geometries", "transform", "children"]

    def __init__(self, geometries=None, transform=np.eye(4)):
        if geometries is None:
            geometries = []
        self.geometries = geometries
        self.transform = transform
        self.children = defaultdict(lambda: LazyTree())

    def __getitem__(self, item):
        return self.children[item]

    def getdescendant(self, path):
        t = self
        for p in path:
            t = t[p]
        return t

    def descendants(self, prefix=tuple()):
        result = []
        for (key, val) in list(self.children.items()):
            childpath = prefix + (key,)
            result.append(childpath)
            result.extend(val.descendants(childpath))
        return result


class CommandQueue(object):
    def __init__(self):
        self.settransform = set()
        self.setgeometry = set()
        self.delete = set()

    def isempty(self):
        return not (self.settransform or self.setgeometry or self.delete)

    def empty(self):
        self.settransform = set()
        self.setgeometry = set()
        self.delete = set()


class Visualizer(object):
    """
    A Visualizer is a lightweight object that contains a CoreVisualizer and a
    path. The CoreVisualizer does all of the work of storing geometries and
    publishing LCM messages. By storing the path in the Visualizer instance,
    we make it easy to do things like store or pass a Visualizer that draws to
    a sub-part of the viewer tree.
    Many Visualizer objects can all share the same CoreVisualizer.
    """
    __slots__ = ["core", "path"]

    def __init__(self, path=None, lcm=None, core=None):
        if core is None:
            core = CoreVisualizer(lcm)
        if path is None:
            path = tuple()
        else:
            if isinstance(path, str):
                path = tuple(path.split("/"))
                if not path[0]:
                    path = tuple([p for p in path if p])
        self.core = core
        self.path = path

    def setgeometry(self, geomdata):
        """
        Set the geometries at this visualizer's path to the given
        geomdata (replacing whatever was there before).
        geomdata can be any one of:
          * a single BaseGeometry
          * a single GeometryData
          * a collection of any combinations of BaseGeometry and GeometryData
        """
        self.core.setgeometry(self.path, geomdata)
        return self

    def settransform(self, tform):
        """
        Set the transform for this visualizer's path (and, implicitly,
        any descendants of that path).
        tform should be a 4x4 numpy array representing a homogeneous transform
        """
        self.core.settransform(self.path, tform)

    def delete(self):
        """
        Delete the geometry at this visualizer's path.
        """
        self.core.delete(self.path)

    def __getitem__(self, path):
        """
        Indexing into a visualizer returns a new visualizer with the given
        path appended to this visualizer's path.
        """
        return Visualizer(path=self.path + (path,),
                          lcm=self.core.lcm,
                          core=self.core)

    def start_handler(self):
        """
        Start a Python thread that will subscribe to messages from the remote
        viewer and handle those responses. This enables automatic reloading of
        geometry into the viewer if, for example, the viewer is restarted
        later.
        """
        self.core.start_handler()


class CoreVisualizer(object):
    def __init__(self, lcm=None):
        if lcm is None:
            lcm = LCM()
        self.lcm = lcm
        self.client_id = CLIENT_ID_FACTORY.new_client_id()
        self.tree = LazyTree()
        self.queue = CommandQueue()
        self.publish_immediately = True
        self.lcm.subscribe(self._response_channel(),
                           self._handle_response)
        self.handler_thread = None

    def _request_channel(self):
        return "DIRECTOR_TREE_VIEWER_REQUEST_<{:s}>".format(self.client_id)

    def _response_channel(self):
        return "DIRECTOR_TREE_VIEWER_RESPONSE_<{:s}>".format(self.client_id)

    def _handler_loop(self):
        while True:
            self.lcm.handle()

    def start_handler(self):
        if self.handler_thread is not None:
            return
        self.handler_thread = threading.Thread(
            target=self._handler_loop)
        self.handler_thread.daemon = True
        self.handler_thread.start()

    def _handle_response(self, channel, msgdata):
        msg = viewer2_comms_t.decode(msgdata)
        data = json.loads(msg.data.decode())
        if data["status"] == 0:
            pass
        elif data["status"] == 1:
            for path in self.tree.descendants():
                self.queue.setgeometry.add(path)
                self.queue.settransform.add(path)
        else:
            raise ValueError(
                "Unhandled response from viewer: {}".format(msg.data.decode()))

    def setgeometry(self, path, geomdata):
        if isinstance(geomdata, BaseGeometry):
            self._load(path, [GeometryData(geomdata)])
        elif isinstance(geomdata, Iterable):
            self._load(path, geomdata)
        else:
            self._load(path, [geomdata])

    def _load(self, path, geoms):
        converted_geom_data = []
        for geom in geoms:
            if isinstance(geom, GeometryData):
                converted_geom_data.append(geom)
            else:
                converted_geom_data.append(GeometryData(geom))
        self.tree.getdescendant(path).geometries = converted_geom_data
        self.queue.setgeometry.add(path)
        self._maybe_publish()

    def settransform(self, path, tform):
        self.tree.getdescendant(path).transform = tform
        self.queue.settransform.add(path)
        self._maybe_publish()

    def delete(self, path):
        if not path:
            self.tree = LazyTree()
        else:
            t = self.tree.getdescendant(path[:-1])
            if path[-1] in t.children:
                del t.children[path[-1]]
        self.queue.delete.add(path)
        self._maybe_publish()

    def _maybe_publish(self):
        if self.publish_immediately:
            self.publish()

    def publish(self):
        if not self.queue.isempty():
            data = self.serialize_queue()
            msg = to_lcm(data)
            self.lcm.publish(self._request_channel(), msg.encode())
            self.queue.empty()

    def serialize_queue(self):
        delete = []
        setgeometry = []
        settransform = []
        for path in self.queue.delete:
            delete.append({"path": path})
        for path in self.queue.setgeometry:
            geoms = self.tree.getdescendant(path).geometries or []
            setgeometry.append({
                "path": path,
                "geometries": [geom.serialize() for geom in geoms]
            })
        for path in self.queue.settransform:
            settransform.append({
                "path": path,
                "transform": serialize_transform(
                    self.tree.getdescendant(path).transform)
            })
        return {
            "utime": int(time.time() * 1e6),
            "delete": delete,
            "setgeometry": setgeometry,
            "settransform": settransform
        }


if __name__ == '__main__':
    # We can provide an initial path if we want
    vis = Visualizer(path="/root/folder1")

    # Start a thread to handle responses from the viewer. Doing this enables
    # the automatic reloading of missing geometry if the viewer is restarted.
    vis.start_handler()

    vis["boxes"].setgeometry(
        [GeometryData(Box([1, 1, 1]),
         color=np.random.rand(4),
         transform=transformations.translation_matrix([x, -2, 0]))
         for x in range(10)])

    # Index into the visualizer to get a sub-tree. vis.__getitem__ is lazily
    # implemented, so these sub-visualizers come into being as soon as they're
    # asked for
    vis = vis["group1"]

    box_vis = vis["box"]
    sphere_vis = vis["sphere"]

    box = Box([1, 1, 1])
    geom = GeometryData(box, color=[0, 1, 0, 0.5])
    box_vis.setgeometry(geom)

    sphere_vis.setgeometry(Sphere(0.5))
    sphere_vis.settransform(transformations.translation_matrix([1, 0, 0]))

    vis["test"].setgeometry(Triad())
    vis["test"].settransform(transformations.concatenate_matrices(
        transformations.rotation_matrix(1.0, [0, 0, 1]),
        transformations.translation_matrix([-1, 0, 1])))

    vis["triad"].setgeometry(Triad())

    # Setting the geometry preserves the transform at that path.
    # Call settransform(np.eye(4)) if you want to clear the transform.
    vis["test"].setgeometry(Triad())

    # bug, the sphere is loaded and replaces the previous
    # geometry but it is not drawn with the correct color mode
    vis["test"].setgeometry(Sphere(0.5))

    for theta in np.linspace(0, 2 * np.pi, 100):
        vis.settransform(transformations.rotation_matrix(theta, [0, 0, 1]))
        time.sleep(0.01)

    #vis.delete()
