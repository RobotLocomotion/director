import os

basedir = os.path.dirname(__file__)
plugin = os.path.abspath(os.path.join(basedir, 'libmy_library.so'))
assert os.path.isfile(plugin)

_pythonManager.loadPlugin(plugin, 'init_my_library')


import my_module
