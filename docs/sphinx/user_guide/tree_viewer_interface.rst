Remote Tree Viewer Interface
============================

Most of this documentation focuses on developing visualizations from within Director itself, but it is also possible to use Director as a standalone remote-controlled 3D viewer. By sending messages from your application over LCM, you can create and move geometric primitives in the viewer. 

To launch the viewer, run the ``drake-visualizer`` binary which is created by Director. For an example of a Python client, see director/viewerclient.py. The remote tree viewer protocol is described below. The current implementation uses LCM for the message transport and JSON to encode the viewer commands. Future versions may add new encoding methods for better performance. 

Viewer Tree
-----------

The remote viewer is built around the concept of a tree of objects (in other words, an acyclic scene graph). We refer to nodes in the tree by their `path`, that is, by the list of nodes traversed from the root of the tree. We can create and delete 3D objects anywhere in the tree by specifying their paths, and we can set the homogeneous transform of any path. The pose of an object in the viewer is determined by the composition of all transforms in its path. 

For example, if we have a viewer with two objects, a box at path ``robot/box1`` and a box at path ``robot/box2``, our tree will look like::

	-robot
	   |-box1
	   |-box2

If we apply a transform to the path ``robot/box1``, then only box1 will be affected, but if we apply a transform to the path ``robot``, then both boxes will move together. 

Protocol
-------- 

Communication with the viewer happens over two LCM channels: 

:DIRECTOR_TREE_VIEWER_REQUEST_<client_id>: for communication from the client to the viewer
:DIRECTOR_TREE_VIEWER_RESPONSE_<client_id>: for communication from the viewer back to the client

The ``client_id`` can consist of any characters except ``<`` and ``>``, and it should be unique for each client. Note that the ID needs to be enclosed within literal ``<`` and ``>`` characters. 

Two-way communication is not mandatory: a simple client may just publish on ``_REQUEST`` and ignore all responses. But handling responses from the viewer enables better synchronization of the client and viewer state, as discussed in responses_. 

All communication on both channels uses a single general-purpose LCM type: viewer2_comms_t_. The ``viewer2_comms_t`` message contains a byte array of encoded command data along with information about the format of that data. Currently, only one format is defined: ``treeviewer_json`` version 1.0. This format consists of a string of JSON-encoded commands. 

.. _viewer2_comms_t: https://github.com/RobotLocomotion/lcmtypes/blob/master/lcmtypes/viewer2_comms_t.lcm

Sample LCM message::
	
	msg.format = "treeviewer_json"
	msg.format_version_major = 1
	msg.format_version_minor = 0
	msg.data = data_payload
	msg.num_bytes = len(data_payload)

The ``data_payload`` is a JSON-encoded dictionary with four fields:

:timestamp: Microseconds since the Unix epoch and expressed as an integer
:setgeometry: A list of setgeometry_ commands. May be empty.
:settransform: A list of settransform_ commands. May be empty.
:delete: A list of delete_ commands. May be empty. 

.. _setgeometry:

Setgeometry Command
-------------------

The `setgeometry` command sets geometric object at a given path, creating nodes in the viewer tree as necessary. It will overwrite any geometry that was already present at that path in the viewer. A `setgeometry` command is a dictionary with the following keys:

:path: the path for the geometry in the tree as a list of strings
:geometries: a list of geometry descriptions

Each geometry description, in turn, is a dictionary with the key ``type`` that indicates the kind of geometric primitive. Various geometry descriptions have other mandatory or optional fields:

Fields for all geometry types:

:type: (required) a keyword indicating the kind of geometry. Currently supports: ``box``, ``sphere``, ``cylinder``, ``capsule``, ``ellipsoid``, ``mesh_file``, ``mesh_data``, ``pointcloud``, ``planar_lidar``, ``triad``, ``line``
:color: (optional) the geometry's color, in RGBA order, as a list of 4 floating point values between 0 and 1. Defaults to [1, 1, 1, 1]
:transform: (optional) a transform to apply to this geometry, in the same format as used in `settransform`. Defaults to identity. 

------

Box fields:

:lengths: the lengths of the box along each dimension, in XYZ order as a list of 3 floating point values

------

Sphere fields:

:radius: radius of the sphere

------

Cylinder fields:

:radius: radius
:length: length along the Z axis

------

Capsule fields:

:radius: radius of the capsule and the hemispheres at each end
:length: length of the capsule, not including the end caps

------

Ellipsoid Fields:

:radii: the radii of the ellipsoid along the XYZ axes

------

Mesh File Fields:

A ``mesh_file`` geometry type contains a path to a mesh file on disk. 

:filename: the absolute path to the mesh file
:scale: (optional) a scaling factor to apply to the mesh. Defaults to 1.

------

Mesh Data Fields:

A ``mesh_data`` geometry encodes the entire mesh in the message itself.. 

:vertices: the mesh vertices as a list of triplets in XYZ order. 
:faces: a list of triplets describing the faces of the mesh. Each element in the list gives the indices (0-based) of three vertices which are joined as a triangular face. 

------

Pointcloud Fields:

:points: the point coordinates as a list of triplets in XYZ order
:channels: (optional) a dictionary of channel data to attach to the points. Each key in the dict should be the name of the channel, and the corresponding value should be a list of length (number of points). The channel ``rgb`` is special: its value (a list of triplets in RGB order) will be used to color the points by default. 

------

Planar Lidar Fields:

The ``planar_lidar`` type is simply a shortcut for generating a pointcloud without specifying the coordinates of every point. Instead, we only need to specify a list of distances and the angles over which those ranges were measured. 

:ranges: a list of distance measurements
:angle_start: the angle about the Z axis (in rad) corresponding to ``ranges[0]``
:angle_step: the angular spacing between adjacent range measurements (in rad) about the Z axis

------

Line Fields:

The ``line`` type generates a line through a sequence of two or more points. 

:points: the point coordinates as a list of 2 or more triplets in XYZ order.
:radius: (optional) the radius of the line. A radius of 0 creates a hairline. A radius > 0 creates a tube. Defaults to 0.01
:closed: (optional) a boolean indicating whether to add a segment from the last point back to the first. Defaults to False.
:start_head: (optional) a boolean indicating whether to draw an arrowhead at the beginning of the first line segment. Defaults to False.
:end_head: (optional) a boolean indicating whether to draw an arrowhead at the end of the last line segment. Defaults to False.
:head_radius: (optional) radius of the arrowheads. Defaults to 0.05.
:head_length: (optional) length of the arrowheads. Defaults to ``head_radius``

.. _settransform:

Settransform Command
--------------------

The `settransform` command sets the transform of a given path. A command has two fields:

:path: the path, as a list of strings
:transform: the transform, as a dictionary

The format of the ``transform`` field is a dictionary with two keys:

:translation: (optional) the translational component, in [x, y, z] order. Defaults to [0, 0, 0]. 
:quaternion: (optional) the rotational component, in [w, x, y, z] order. Defaults to [1, 0, 0, 0]

.. _delete:

Delete Command
--------------

A delete command has just one field:

:path: the path of the geometry to delete. Any geometries and transforms at this path and its descendants will be deleted. 

Examples
--------

Create a box at path "robot1/link1"::

	{
		"timestamp": 1486691399249288,
		"setgeometry": [
			{
				"path": ["robot1", "link1"],
				"geometry": {
					"type": "box",
					"color": [1, 0, 0, 0.5],
					"lengths": [1, 0.5, 2]
				}
			}
		],
		"settransform": [],
		"delete": []
	}

------

Create a sphere at path "robot1/link2" and a pointcloud at "perception/cloud1"::

	{
		"timestamp": 1486691399249288,
		"setgeometry": [
			{
				"path": ["robot1", "link2"],
				"geometry": {
					"type": "sphere",
					"radius": 0.5
				}
			},
			{
				"path": ["perception", "cloud1"],
				"geometry": {
					"type": "pointcloud",
					"points": [[0.0, 0, 0],
					           [0.1, 0, 0],
					           [0.2, 0, 0],
					           [0.3, 0, 0]]
					"channels": {
						"rgb": [[1, 1, 1],  # white
						        [1, 0, 0],  # red
						        [0, 1, 0],  # green
						        [0, 0, 1]]  # blue
					}
				}
			},
		],
		"settransform": [],
		"delete": []
	}

------

Translate the entire ``robot`` group 1 meter in Z::

	{
		"setgeometry": [],
		"settransform":
			{
				"path": ["robot1"],
				"transform": {
					"translation": [0, 0, 1],
					"quaternion": [1, 0, 0, 0]
				}
			}
		],
		"delete": []
	}

------

Delete the sphere at ``robot1/link2``::

	{
		"setgeometry": [],
		"settransform": []
		"delete": [
			{
				"path": ["robot1", "link2"]
			}
		]
	}

.. _responses:

Handling Responses 
------------------

When the viewer receives a request, it will respond with a message on the ``DIRECTOR_TREE_VIEWER_RESPONSE`` channel. This message will be in the same format as the request: an LCM message containing JSON-encoded data. 

The viewer response data is a dictionary with at least one field. It will always contain the field ``status`` with an integer value. The following status values are defined:

:status == 0: OK
:status == 1: MISSING_PATHS
:status == -1: ERROR_UNKNOWN_FORMAT
:status == -2: ERROR_UNKNOWN_FORMAT_VERSION

Status MISSING_PATHS means that the viewer received a settransform_ command for a path which has no geometry at that path or any of its descendants. This can happen if, for example, the client sends a settransform_ before sending a setgeometry_, or if the viewer is restarted and loses its state. A MISSING_PATHS status will be accompanied by a field ``missing_paths`` in the viewer response data, listing all of the paths which were not found. The client should send the appropriate setgeometry_ commands for those paths. 

