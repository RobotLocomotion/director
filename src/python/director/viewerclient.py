from __future__ import absolute_import, division, print_function

import time
import json
from collections import defaultdict, namedtuple, Iterable
import numpy as np
from lcm import LCM
from robotlocomotion import viewer2_comms_t
from director.thirdparty import transformations


def to_lcm(data):
    msg = viewer2_comms_t()
    msg.utime = data["utime"]
    msg.format = "treeviewer_json"
    msg.format_version_major = 1
    msg.format_version_minor = 0
    msg.data = json.dumps(data)
    msg.num_bytes = len(msg.data)
    return msg


def serialize_transform(tform):
    return {
        "translation": list(transformations.translation_from_matrix(tform)),
        "quaternion": list(transformations.quaternion_from_matrix(tform))
    }


class GeometryData(object):
    __slots__ = ["geometry", "color", "transform"]

    def __init__(self, geometry, color=[1, 0, 0, 0.5], transform=np.eye(4)):
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


class Box(BaseGeometry, namedtuple("Box", ["lengths"])):
    def serialize(self):
        return {
            "type": "box",
            "lengths": list(self.lengths)
        }


class Sphere(BaseGeometry, namedtuple("Sphere", ["radius"])):
    def serialize(self):
        return {
            "type": "sphere",
            "radius": self.radius
        }


class Ellipsoid(BaseGeometry, namedtuple("Ellipsoid", ["radii"])):
    def serialize(self):
        return {
            "type": "ellipsoid",
            "radii": list(self.radii)
        }


class Cylinder(BaseGeometry, namedtuple("Cylinder", ["length", "radius"])):
    def serialize(self):
        return {
            "type": "cylinder",
            "length": self.length,
            "radius": self.radius
        }


class Triad(BaseGeometry, namedtuple("Triad", [])):
    def serialize(self):
        return {
            "type": "triad"
        }


class LazyTree(object):
    __slots__ = ["geometries", "transform", "children"]

    def __init__(self, geometries=[], transform=np.eye(4)):
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


class CommandQueue(object):
    def __init__(self):
        self.draw = set()
        self.load = set()
        self.delete = set()

    def isempty(self):
        return not (self.draw or self.load or self.delete)

    def empty(self):
        self.draw = set()
        self.load = set()
        self.delete = set()


class Visualizer(object):
    __slots__ = ["core", "path"]

    def __init__(self, core=None, path=None, lcm=None):
        if core is None:
            if lcm is None:
                lcm = LCM()
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

    def load(self, geomdata):
        self.core.load(self.path, geomdata)
        return self

    def draw(self, tform):
        self.core.draw(self.path, tform)

    def delete(self):
        self.core.delete(self.path)

    def __getitem__(self, path):
        return Visualizer(core=self.core,
                          path=self.path + (path,),
                          lcm=self.core.lcm)


class CoreVisualizer(object):
    def __init__(self, lcm=LCM()):
        self.lcm = lcm
        self.tree = LazyTree()
        self.queue = CommandQueue()
        self.publish_immediately = True
        self.lcm.subscribe("DIRECTOR_TREE_VIEWER_RESPONSE",
                           self.handle_response)

    def handle_response(self, channel, msgdata):
        msg = viewer2_comms_t.decode(msgdata)
        print(msg)

    def load(self, path, geomdata):
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
        self.queue.load.add(path)
        self._maybe_publish()

    def draw(self, path, tform):
        self.tree.getdescendant(path).transform = tform
        self.queue.draw.add(path)
        self._maybe_publish()

    def delete(self, path):
        if not path:
            self.tree = LazyTree()
        else:
            t = self.tree.getdescendant(path[:-1])
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
            self.lcm.publish("DIRECTOR_TREE_VIEWER_REQUEST", msg.encode())
            self.queue.empty()

    def serialize_queue(self):
        delete = []
        load = []
        draw = []
        for path in self.queue.delete:
            delete.append({"path": path})
        for path in self.queue.load:
            geoms = self.tree.getdescendant(path).geometries
            if geoms:
                load.append({
                    "path": path,
                    "geometries": [geom.serialize() for geom in geoms]
                })
        for path in self.queue.draw:
            draw.append({
                "path": path,
                "transform": serialize_transform(
                    self.tree.getdescendant(path).transform)
            })
        return {
            "utime": int(time.time() * 1e6),
            "delete": delete,
            "load": load,
            "draw": draw
        }


if __name__ == '__main__':
    # We can provide an initial path if we want
    vis = Visualizer(path="/root/folder1")

    vis["boxes"].load(
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
    box_vis.load(geom)

    sphere_vis.load(Sphere(1.0))
    sphere_vis.draw(transformations.translation_matrix([1, 0, 0]))

    for theta in np.linspace(0, 2 * np.pi, 100):
        vis.draw(transformations.rotation_matrix(theta, [0, 0, 1]))
        time.sleep(0.01)
    vis.delete()
