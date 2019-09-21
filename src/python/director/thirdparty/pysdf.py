

import itertools
import os
import xml.etree.ElementTree as ET
import xml.dom.minidom

import numpy
from director.thirdparty import transformations
from .naming import *
from .conversions import *

models_path = os.path.expanduser('~/.gazebo/models/')
catkin_ws_path = os.path.expanduser('~') + '/catkin_ws/src/'
supported_sdf_versions = [1.4, 1.5]



def find_mesh_in_catkin_ws(filename):
  if not find_mesh_in_catkin_ws.cache:
    result = ''
    for root, dirs, files in os.walk(catkin_ws_path, followlinks=True):
      for currfile in files:
        if currfile.endswith('.stl') or currfile.endswith('.dae'):
          partial_path = ''
          for path_part in root.split('/'):
            partial_path += path_part + '/'
            if os.path.exists(partial_path + '/package.xml'):
              break
          catkin_stack_path = partial_path.replace(path_part + '/', '')
          filename_path = os.path.join(root, currfile).replace(catkin_stack_path, '')
          find_mesh_in_catkin_ws.cache.append(filename_path)
  matching = [path for path in find_mesh_in_catkin_ws.cache if filename in path]
  return ' OR '.join(matching)
find_mesh_in_catkin_ws.cache = []


def find_model_in_gazebo_dir(modelname):
  if not find_model_in_gazebo_dir.cache:
    for root, dirs, files in os.walk(models_path, followlinks=True):
      for currfile in files:
        if currfile != 'model.sdf':
          continue
        filename_path = os.path.join(root, currfile)
        tree = ET.parse(filename_path)
        root = tree.getroot()
        if root.tag != 'sdf':
          continue
        modelnode = get_node(root, 'model')
        if modelnode == None:
          continue
        modelname_in_file = modelnode.attrib['name']
        find_model_in_gazebo_dir.cache[modelname_in_file] = filename_path
    #print(find_model_in_gazebo_dir.cache)
  return find_model_in_gazebo_dir.cache.get(modelname)
find_model_in_gazebo_dir.cache = {}


def pose2origin(node, pose):
  xyz, rpy = homogeneous2translation_rpy(pose)
  ET.SubElement(node, 'origin', {'xyz': array2string(rounded(xyz)), 'rpy': array2string(rounded(rpy))})


def prettyXML(uglyXML):
  return xml.dom.minidom.parseString(uglyXML).toprettyxml(indent='  ')


def get_tag(node, tagname, default = None):
  tag = node.findall(tagname)
  if tag:
    return tag[0].text
  else:
    return default


def get_node(node, tagname, default = None):
  tag = node.findall(tagname)
  if tag:
    return tag[0]
  else:
    return default


def get_tag_pose(node):
  pose = get_tag(node, 'pose', '0 0 0  0 0 0')
  return pose_string2homogeneous(pose)


def indent(string, spaces):
  return string.replace('\n', '\n' + ' ' * spaces).strip()


def model_from_include(parent, include_node):
    submodel_uri = get_tag(include_node, 'uri')
    submodel_path = submodel_uri.replace('model://', models_path) + os.path.sep + 'model.sdf'
    submodel_name = get_tag(include_node, 'name')
    submodel_pose = get_tag_pose(include_node)
    return Model(parent, name=submodel_name, pose=submodel_pose, file=submodel_path)


def homogeneous_times_vector(homogeneous, vector):
  vector_as_hom = transformations.identity_matrix()
  vector_as_hom[:3,3] = vector.T
  res = numpy.dot(homogeneous, vector_as_hom)
  return res[:3,3].T 





class SDF(object):
  def __init__(self, **kwargs):
    self.world = World()
    if 'file' in kwargs:
      self.from_file(kwargs['file'])
    elif 'model' in kwargs:
      self.from_model(kwargs['model'])


  def from_file(self, filename):
    if not os.path.exists(filename):
      print('Failed to open SDF because %s does not exist' % filename)
      return
    tree = ET.parse(filename)
    root = tree.getroot()
    if root.tag != 'sdf':
      print('Not a SDF file. Aborting.')
      return
    self.version = float(root.attrib['version'])
    if not self.version in supported_sdf_versions:
      print('Unsupported SDF version in %s. Aborting.\n' % filename)
      return
    self.world.from_tree(root, version=self.version)


  def from_model(self, modelname):
    sdf_file = models_path + modelname + '/model.sdf'
    if not os.path.exists(sdf_file):
      sdf_file = find_model_in_gazebo_dir(modelname)
    if not sdf_file:
      print('Could not resolve modelname=%s to its SDF file in %s' % (modelname, models_path))
      return
    self.from_file(sdf_file)



class World(object):
  def __init__(self, **kwargs):
    self.name = '__default__'
    self.models = []
    self.lights = []
    self.version = kwargs.get('version', 0.0)


  def from_tree(self, node, **kwargs):
    self.version = kwargs.get('version', self.version)
    if node.findall('world'):
      node = node.findall('world')[0]
      for include_node in node.iter('include'):
        self.models.append(model_from_include(None, include_node))
      # TODO lights
    self.models += [Model(tree=model_node, version=self.version) for model_node in node.findall('model')]


  def plot_to_file(self, plot_filename):
    import pygraphviz as pgv
    graph = pgv.AGraph(directed=True)
    self.plot(graph)
    graph.draw(plot_filename, prog='dot')


  def plot(self, graph):
    graph.add_node('world')

    for model in self.models:
      model.plot(graph)
      graph.add_edge('world', model.name + '::' + model.root_link.name)


  def get_link(self, requested_linkname):
    #print('World.get_link: rl=%s' % requested_linkname)
    for model in self.models:
      link = model.get_link(requested_linkname, model.name)
      if link:
        return link


  def for_all_links(self, func, **kwargs):
    for model in self.models:
      model.for_all_links(func, **kwargs)


  def for_all_joints(self, func, **kwargs):
    for model in self.models:
      model.for_all_joints(func, **kwargs)


  def for_all_submodels(self, func, **kwargs):
    for model in self.models:
      model.for_all_submodels(func, **kwargs)



class SpatialEntity(object):
  def __init__(self, **kwargs):
    self.name = ''
    self.pose = transformations.identity_matrix()
    self.pose_world = transformations.identity_matrix()


  def __repr__(self):
    return ''.join((
      'name: %s\n' % self.name,
      'pose: %s\n' % homogeneous2tq_string_rounded(self.pose),
      'pose_world: %s\n' % homogeneous2tq_string_rounded(self.pose_world),
    ))


  def from_tree(self, node, **kwargs):
    if node == None:
      return
    self.name = node.attrib['name']
    self.pose = get_tag_pose(node)



class Model(SpatialEntity):
  def __init__(self, parent_model = None, **kwargs):
    super(Model, self).__init__(**kwargs)
    self.parent_model = parent_model
    self.version = kwargs.get('version', 0.0)
    self.submodels = []
    self.links = []
    self.joints = []
    self.root_link = None
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'], **kwargs)
    elif 'file' in kwargs:
      self.from_file(kwargs['file'], **kwargs)

    if not self.parent_model:
      self.root_link = self.find_root_link()
      self.build_tree()
      self.calculate_absolute_pose()


  def __repr__(self):
    return ''.join((
      'Model(\n', 
      '  %s\n' % indent(super(Model, self).__repr__(), 2),
      '  version: %s\n' % self.version,
      '  root_link: %s\n' % self.root_link.name if self.root_link else '',
      '  links:\n',
      '    %s' % '\n    '.join([indent(str(l), 4) for l in self.links]),
      '\n',
      '  joints:\n',
      '    %s' % '\n    '.join([indent(str(j), 4) for j in self.joints]),
      '\n',
      '  submodels:\n',
      '    %s' % '\n    '.join([indent(str(m), 4) for m in self.submodels]),
      '\n',
      ')'
    ))


  def from_file(self, filename, **kwargs):
    if not os.path.exists(filename):
      print('Failed to open Model because %s does not exist' % filename)
      return
    tree = ET.parse(filename)
    root = tree.getroot()
    if root.tag != 'sdf':
      print('Not a SDF file. Aborting.')
      return
    self.version = float(root.attrib['version'])
    if not self.version in supported_sdf_versions:
      print('Unsupported SDF version in %s. Aborting.\n' % filename)
      return
    modelnode = get_node(root, 'model')
    self.from_tree(modelnode, **kwargs)

    # Override name (e.g. for <include>)
    kwargs_name = kwargs.get('name')
    if kwargs_name:
      self.name = kwargs_name

    # External pose offset (from <include>)
    self.pose = numpy.dot(kwargs.get('pose', transformations.identity_matrix()), self.pose)


  def from_tree(self, node, **kwargs):
    if node == None:
      return
    if node.tag != 'model':
      print('Invalid node of type %s instead of model. Aborting.' % node.tag)
      return
    self.version = kwargs.get('version', self.version)
    super(Model, self).from_tree(node, **kwargs)
    self.links = [Link(self, tree=link_node) for link_node in node.iter('link')]
    self.joints = [Joint(self, tree=joint_node) for joint_node in node.iter('joint')]

    for include_node in node.iter('include'):
      self.submodels.append(model_from_include(self, include_node))


  def add_urdf_elements(self, node, prefix = ''):
    full_prefix = prefix + '::' + self.name if prefix else self.name
    for entity in self.joints + self.links:
      entity.add_urdf_elements(node, full_prefix)
    for submodel in self.submodels:
      submodel.add_urdf_elements(node, full_prefix)


  def to_urdf_string(self):
    urdfnode = ET.Element('robot', {'name': self.name})
    self.add_urdf_elements(urdfnode)
    return ET.tostring(urdfnode)


  def save_urdf(self, filename):
    urdf_file = open(filename, 'w')
    pretty_urdf_string = prettyXML(self.to_urdf_string())
    urdf_file.write(pretty_urdf_string)
    urdf_file.close()


  def get_joint(self, requested_jointname, prefix = ''):
    #print('get_joint: n=%s rj=%s p=%s' % (self.name, requested_jointname, prefix))
    full_prefix = prefix + '::' if prefix else ''
    for joint in self.joints:
      if full_prefix + joint.name == requested_jointname:
        return joint
    for submodel in self.submodels:
      res = submodel.get_joint(requested_jointname, submodel.name)
      if res:
        return res


  def get_link(self, requested_linkname, prefix = ''):
    #print('Model.get_link: n=%s rl=%s p=%s' % (self.name, requested_linkname, prefix))
    full_prefix = prefix + '::' if prefix else ''
    for link in self.links:
      if full_prefix + link.name == requested_linkname:
        return link
    for submodel in self.submodels:
      res = submodel.get_link(requested_linkname, prefix + '::' + submodel.name if prefix else submodel.name)
      if res:
        return res


  def build_tree(self):
    for joint in self.joints:
      joint.tree_parent_link = self.get_link(joint.parent)
      if not joint.tree_parent_link:
        print('Failed to find parent %s of joint %s. Aborting' % (joint.parent, joint.name))
      joint.tree_child_link = self.get_link(joint.child)
      if not joint.tree_child_link:
        print('Failed to find child %s of joint %s. Aborting' % (joint.child, joint.name))
        return None
      joint.tree_parent_link.tree_child_joints.append(joint)
      joint.tree_child_link.tree_parent_joint = joint
    for submodel in self.submodels:
      submodel.build_tree()


  def calculate_absolute_pose(self, worldMVparent = transformations.identity_matrix()):
    worldMVmodel = transformations.concatenate_matrices(worldMVparent, self.pose)
    self.pose_world = worldMVmodel
    for submodel in self.submodels:
      submodel.calculate_absolute_pose(worldMVmodel)
    for link in self.links:
      link.pose_world = transformations.concatenate_matrices(worldMVmodel, link.pose)
    for joint in self.joints:
      joint.pose_world = transformations.concatenate_matrices(joint.tree_child_link.pose_world, joint.pose)


  def find_root_link(self):
    if not self.links:
      print('Model %s has no links and therefore no root link. Aborting' % self.name)
      return None
    curr_link = self.links[0]
    while True:
      parent_link = self.get_parent(curr_link.name)
      if not parent_link:
        return curr_link
      curr_link = parent_link


  def get_parent(self, requested_linkname, prefix = '', visited = []):
    if self.name in visited:
      return None
    full_prefix = prefix + '::' if prefix else ''
    for joint in self.joints:
      if joint.child == full_prefix + requested_linkname:
        return self.get_link(joint.parent)
    # Parent links can also be in submodels
    for submodel in self.submodels:
      res = submodel.get_parent(requested_linkname, full_prefix + submodel.name, visited + [self.name])
      if res:
        return res
    if self.parent_model:
      return self.parent_model.get_parent(requested_linkname, self.name, visited + [self.name])


  def plot(self, graph, prefix = ''):
    full_prefix = prefix + '::' + self.name if prefix else self.name
    full_prefix += '::'
    for link in self.links:
      graph.add_node(full_prefix + link.name, label=full_prefix + link.name + '\\nrel: ' + homogeneous2tq_string_rounded(link.pose) + '\\nabs: ' + homogeneous2tq_string_rounded(link.pose_world))
    subgraph = graph.add_subgraph([full_prefix + link.name for link in self.links], 'cluster_' + self.name, color='gray', label=self.name + '\\nrel: ' + homogeneous2tq_string_rounded(self.pose) + '\\nabs: ' + homogeneous2tq_string_rounded(self.pose_world))

    for submodel in self.submodels:
      submodel.plot(graph, full_prefix.rstrip('::'))

    for joint in self.joints:
      graph.add_edge(full_prefix + joint.parent, full_prefix + joint.child, label=full_prefix + joint.name + '\\nrel: ' + homogeneous2tq_string_rounded(joint.pose) + '\\nabs: ' + homogeneous2tq_string_rounded(joint.pose_world))


  def get_root_model(self):
    curr_model = self
    while True:
      if not curr_model.parent_model:
        return curr_model
      curr_model = curr_model.parent_model


  def for_all_links(self, func, prefix = '', **kwargs):
    full_prefix = prefix + '::' + self.name if prefix else self.name
    for link in self.links:
      func(link, full_prefix + '::' + link.name, **kwargs)
    for submodel in self.submodels:
      submodel.for_all_links(func, full_prefix, **kwargs)


  def for_all_joints(self, func, prefix = '', **kwargs):
    full_prefix = prefix + '::' + self.name if prefix else self.name
    for joint in self.joints:
      func(joint, full_prefix + '::' + joint.name, **kwargs)
    for submodel in self.submodels:
      submodel.for_all_joints(func, full_prefix, **kwargs)


  def for_all_submodels(self, func, prefix = '', **kwargs):
    full_prefix = prefix + '::' + self.name if prefix else self.name
    func(self, full_prefix, **kwargs)
    for submodel in self.submodels:
      submodel.for_all_submodels(func, full_prefix, **kwargs)


  def get_full_name(self):
    name = self.name
    curr_model = self
    while True:
      if not curr_model.parent_model:
        break
      curr_model = curr_model.parent_model
      name = curr_model.name + '::' + name
    return name



class Link(SpatialEntity):
  def __init__(self, parent_model, **kwargs):
    super(Link, self).__init__(**kwargs)
    self.parent_model = parent_model
    self.gravity = True
    self.inertial = Inertial()
    self.collision = Collision()
    self.visual = Visual()
    self.collisions = []
    self.visuals = []
    self.tree_parent_joint = None
    self.tree_child_joints = []
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Link(\n',
      '  %s\n' % indent(super(Link, self).__repr__(), 2),
      '  %s\n' % indent(str(self.inertial), 2),
      '  collisions:\n',
      '    %s' % '\n    '.join([indent(str(l), 4) for l in self.collisions]),
      '  visuals:\n',
      '    %s' % '\n    '.join([indent(str(l), 4) for l in self.visuals]),
      ')'
    ))


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'link':
      print('Invalid node of type %s instead of link. Aborting.' % node.tag)
      return
    super(Link, self).from_tree(node)
    self.inertial = Inertial(tree=get_node(node, 'inertial'))
    self.collisions = [Collision(tree=link_node) for link_node in node.iter('collision')] 
    self.collision = Collision(tree=get_node(node, 'collision'))
    self.visual = Visual(tree=get_node(node, 'visual'))
    self.visuals = [Visual(tree=link_node) for link_node in node.iter('visual')] 

  def is_empty(self):
    return not self.visual.geometry_type and not self.collision.geometry_type

  def add_urdf_elements(self, node, prefix):
    full_prefix = prefix + '::' if prefix else ''
    linknode = ET.SubElement(node, 'link', {'name': sdf2tfname(full_prefix + self.name)})
    # urdf links do not have a coordinate system themselves, only their parts (inertial, collision, visual) have one
    if self.tree_parent_joint:
      if self.tree_parent_joint.parent_model == self.parent_model:
        urdf_pose = transformations.concatenate_matrices(inverse_matrix(self.tree_parent_joint.pose_world), self.pose_world)
      else: # joint crosses includes
        urdf_pose = transformations.identity_matrix()
    else: # root
      urdf_pose = self.pose_world
    self.inertial.add_urdf_elements(linknode, urdf_pose)
    self.collision.add_urdf_elements(linknode, prefix, urdf_pose)
    self.visual.add_urdf_elements(linknode, prefix, urdf_pose)


  def get_full_name(self):
    return self.parent_model.get_full_name() + '::' + self.name
    



class Joint(SpatialEntity):
  def __init__(self, parent_model, **kwargs):
    super(Joint, self).__init__(**kwargs)
    self.parent_model = parent_model
    self.type = ''
    self.parent = ''
    self.child = ''
    self.axis = Axis(self)
    self.axis2 = None
    self.tree_parent_link = None
    self.tree_child_link = None
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Joint(\n',
      '  %s\n' % indent(super(Joint, self).__repr__(), 2),
      '  type: %s\n' % self.type,
      '  parent: %s\n' % self.parent,
      '  child: %s\n' % self.child,
      '  axis: %s\n' % self.axis,
      '  axis2: %s\n' % self.axis2 if self.axis2 else '',
      ')'
    ))


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'joint':
      print('Invalid node of type %s instead of joint. Aborting.' % node.tag)
      return
    super(Joint, self).from_tree(node)
    self.type = node.attrib['type']
    self.parent = get_tag(node, 'parent', '')
    self.child = get_tag(node, 'child', '')
    self.axis = Axis(self, tree=get_node(node, 'axis'))
    if get_node(node, 'axis2') != None:
      self.axis2 = Axis(self, tree=get_node(node, 'axis2'))


  def add_urdf_elements(self, node, prefix):
    full_prefix = prefix + '::' if prefix else ''
    jointnode = ET.SubElement(node, 'joint', {'name': sdf2tfname(full_prefix + self.name)})
    parentnode = ET.SubElement(jointnode, 'parent', {'link': sdf2tfname(full_prefix + self.parent)})
    childnode = ET.SubElement(jointnode, 'child', {'link': sdf2tfname(full_prefix + self.child)})
    # in SDF a joint's pose is given in child link frame, in URDF joint frame = child link frame, i.e. specifiy relative to parent joint (not parent link)
    parent_pose_world = self.tree_parent_link.tree_parent_joint.pose_world if self.tree_parent_link.tree_parent_joint else transformations.identity_matrix()
    if self.tree_parent_link.parent_model == self.parent_model:
      pose2origin(jointnode, transformations.concatenate_matrices(inverse_matrix(parent_pose_world), self.pose_world))
    else: # joint crosses includes
      pose2origin(jointnode, transformations.concatenate_matrices(inverse_matrix(parent_pose_world), self.tree_child_link.pose_world))
    if self.type == 'revolute' and self.axis.lower_limit == 0 and self.axis.upper_limit == 0:
      jointnode.attrib['type'] = 'fixed'
    elif self.type == 'universal':
      # Simulate universal robot as
      # self.parent -> revolute joint (self) -> dummy link -> revolute joint -> self.child
      jointnode.attrib['type'] = 'revolute'
      dummylinknode = ET.SubElement(node, 'link', {'name': sdf2tfname(full_prefix + self.name + '::revolute_dummy_link')})
      childnode.attrib['link'] = dummylinknode.attrib['name']
      dummyjointnode = ET.SubElement(node, 'joint', {'name': sdf2tfname(full_prefix + self.name + '::revolute_dummy_joint')})
      ET.SubElement(dummyjointnode, 'parent', {'link': dummylinknode.attrib['name']})
      ET.SubElement(dummyjointnode, 'child', {'link': sdf2tfname(full_prefix + self.child)})
      dummyjointnode.attrib['type'] = 'revolute'
      self.axis2.add_urdf_elements(dummyjointnode, transformations.concatenate_matrices(inverse_matrix(self.pose_world), self.parent_model.pose_world))
    else:
      jointnode.attrib['type'] = self.type
    #print('self.pose_world\n', self.pose_world)
    #print('self.parent_model.pose_world\n', self.parent_model.pose_world)
    self.axis.add_urdf_elements(jointnode, transformations.concatenate_matrices(inverse_matrix(self.pose_world), self.parent_model.pose_world))


  def get_full_name(self):
    return self.parent_model.get_full_name() + '::' + self.name




class Axis(object):
  def __init__(self, joint, **kwargs):
    self.joint = joint
    self.version = self.joint.parent_model.version
    self.xyz = numpy.array([0, 0, 0])
    self.lower_limit = 0
    self.upper_limit = 0
    self.effort_limit = 0
    self.velocity_limit = 0
    if self.version >= 1.5:
      self.use_parent_model_frame = False
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return 'Axis(xyz=%s,%s lower_limit=%s, upper_limit=%s, effort=%s, velocity=%s)' % (self.xyz, ' use_parent_model_frame=%s,' % self.use_parent_model_frame if self.version >= 1.5 else '', self.lower_limit, self.upper_limit, self.effort_limit, self.velocity_limit)


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'axis' and node.tag != 'axis2':
      print('Invalid node of type %s instead of axis(2). Aborting.' % node.tag)
      return
    self.xyz = numpy.array(get_tag(node, 'xyz').split())
    self.use_parent_model_frame = bool(get_tag(node, 'use_parent_model_frame'))
    limitnode = get_node(node, 'limit')
    if limitnode == None:
      print('limit Tag missing from joint. Aborting.')
      return
    self.lower_limit = float(get_tag(limitnode, 'lower', 0))
    self.upper_limit = float(get_tag(limitnode, 'upper', 0))
    self.effort_limit = float(get_tag(limitnode, 'effort', 0))
    self.velocity_limit = float(get_tag(limitnode, 'velocity', 0))


  def add_urdf_elements(self, node, modelCBTjoint):
    if (self.version <= 1.4) or (self.version >= 1.5 and self.use_parent_model_frame): # SDF 1.4 axis is specified in model frame
      rotation_modelCBTjoint = rotation_only(modelCBTjoint)
      xyz_joint = homogeneous_times_vector(rotation_modelCBTjoint, self.xyz)
      xyz_joint /= numpy.linalg.norm(xyz_joint)
      #print('self.xyz=%s\nmodelCBTjoint:\n%s\nrotation_modelCBT_joint:\n%s\nxyz_joint=%s' % (self.xyz, modelCBTjoint, rotation_modelCBTjoint, xyz_joint))
    else: # SDF 1.5 axis is specified in joint frame unless the use_parent_model_frame flag is set to true
      print('UNTESTED')
      xyz_joint = self.xyz
    axisnode = ET.SubElement(node, 'axis', {'xyz': array2string(rounded(xyz_joint))})
    limitnode = ET.SubElement(node, 'limit', {'lower': str(self.lower_limit), 'upper': str(self.upper_limit), 'effort': str(self.effort_limit), 'velocity': str(self.velocity_limit)})



class Inertial(object):
  def __init__(self, **kwargs):
    self.pose = transformations.identity_matrix()
    self.mass = 0
    self.inertia = Inertia()
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return ''.join((
      'Inertial(\n',
      '  pose: %s\n' % homogeneous2tq_string_rounded(self.pose),
      '  mass: %s\n' % self.mass,
      '  inertia: %s\n' % self.inertia,
      ')'
    ))


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'inertial':
      print('Invalid node of type %s instead of inertial. Aborting.' % node.tag)
      return
    self.pose = get_tag_pose(node)
    self.mass = get_tag(node, 'mass', 0)
    self.inertia = Inertia(tree=get_node(node, 'inertia'))


  def add_urdf_elements(self, node, link_pose):
    inertialnode = ET.SubElement(node, 'inertial')
    massnode = ET.SubElement(inertialnode, 'mass', {'value': str(self.mass)})
    pose2origin(inertialnode, transformations.concatenate_matrices(link_pose, self.pose))
    self.inertia.add_urdf_elements(inertialnode)



class Inertia(object):
  def __init__(self, **kwargs):
    self.ixx = 0
    self.ixy = 0
    self.ixz = 0
    self.iyy = 0
    self.iyz = 0
    self.izz = 0
    self.coords = 'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def __repr__(self):
    return 'Inertia(ixx=%s, ixy=%s, ixz=%s, iyy=%s, iyz=%s, izz=%s)' % (self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz)


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'inertia':
      print('Invalid node of type %s instead of inertia. Aborting.' % node.tag)
      return
    for coord in self.coords:
      setattr(self, coord, get_tag(node, coord, 0))


  def add_urdf_elements(self, node):
    inertianode = ET.SubElement(node, 'inertia')
    for coord in self.coords:
      inertianode.attrib[coord] = str(getattr(self, coord))



class LinkPart(SpatialEntity):
  def __init__(self, **kwargs):
    super(LinkPart, self).__init__(**kwargs)
    self.geometry_type = None
    self.geometry_data = {}
    self.gtypes = 'box', 'cylinder', 'sphere', 'mesh'
    self.color = [ 0.8, 0.8, 0.8, 1 ]  # RGBA
    if 'tree' in kwargs:
      self.from_tree(kwargs['tree'])


  def from_tree(self, node):
    if node == None:
      return
    if node.tag != 'visual' and node.tag != 'collision':
      print('Invalid node of type %s instead of visual or collision. Aborting.' % node.tag)
      return
    super(LinkPart, self).from_tree(node)
    gnode = get_node(node, 'geometry')
    if gnode == None:
      return
    material = get_node(node, 'material')
    if material is not None:
      color_node = get_node(material, 'color')
      if color_node is not None:
        try:
          color = color_node.attrib['rgba'].split(' ')
          self.color = [ float(e) for e in color ]
        except:
          pass
    for gtype in self.gtypes:
      typenode = get_node(gnode, gtype)
      if typenode != None:
        self.geometry_type = gtype
        if gtype == 'box':
          self.geometry_data = {'size': get_tag(typenode, 'size')}
        elif gtype == 'cylinder':
          self.geometry_data = {'radius': get_tag(typenode, 'radius'), 'length': get_tag(typenode, 'length')}
        elif gtype == 'sphere':
          self.geometry_data = {'radius': get_tag(typenode, 'radius')}
        elif gtype == 'mesh':
          self.geometry_data = {'uri': get_tag(typenode, 'uri'), 'scale': get_tag(typenode, 'scale', '1.0 1.0 1.0')}


  def __repr__(self):
    return '%s geometry_type: %s, geometry_data: %s' % (super(LinkPart, self).__repr__().replace('\n', ', ').strip(), self.geometry_type, self.geometry_data)


  def add_urdf_elements(self, node, prefix, link_pose, part_type):
    if not self.geometry_type:
      return
    partnode = ET.SubElement(node, part_type, {'name': sdf2tfname(prefix + '::' + self.name)})
    pose2origin(partnode, transformations.concatenate_matrices(link_pose, self.pose))
    geometrynode = ET.SubElement(partnode, 'geometry')
    if self.geometry_type == 'box':
      boxnode = ET.SubElement(geometrynode, 'box', {'size': self.geometry_data['size']})
    elif self.geometry_type == 'cylinder':
      cylindernode = ET.SubElement(geometrynode, 'cylinder', {'radius': self.geometry_data['radius'], 'length': self.geometry_data['length']})
    elif self.geometry_type == 'sphere':
      spherenode = ET.SubElement(geometrynode, 'sphere', {'radius': self.geometry_data['radius']})
    elif self.geometry_type == 'mesh':
      mesh_file = '/'.join(self.geometry_data['uri'].split('/')[3:])
      mesh_found = find_mesh_in_catkin_ws(mesh_file)
      if mesh_found:
        mesh_path = 'package://' + mesh_found
      else:
        print('Could not find mesh %s in %s' % (mesh_file, catkin_ws_path))
        mesh_path = 'package://PATHTOMESHES/' + mesh_file
      meshnode = ET.SubElement(geometrynode, 'mesh', {'filename': mesh_path, 'scale': self.geometry_data['scale']})



class Collision(LinkPart):
  def __init__(self, **kwargs):
    super(Collision, self).__init__(**kwargs)


  def __repr__(self):
    return 'Collision(%s)' % super(Collision, self).__repr__()


  def add_urdf_elements(self, node, prefix, link_pose):
    super(Collision, self).add_urdf_elements(node, prefix, link_pose, 'collision')



class Visual(LinkPart):
  def __init__(self, **kwargs):
    super(Visual, self).__init__(**kwargs)


  def __repr__(self):
    return 'Visual(%s)' % super(Visual, self).__repr__()


  def add_urdf_elements(self, node, prefix, link_pose):
    super(Visual, self).add_urdf_elements(node, prefix, link_pose, 'visual')

