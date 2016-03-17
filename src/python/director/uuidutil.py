
# Workaround for C runtime issue from Python's uuid module.
# This seems to be a problem with the software built using VS 2013
# against the official Python precompiled binaries.
# http://blender.stackexchange.com/questions/7665/pythons-uuid-module-cause-c-runtime-error-in-ms-windows
def uuid_workaround():
    import platform
    if platform.system() == "Windows":
        import ctypes
        CDLL = ctypes.CDLL
        ctypes.CDLL = None
        import uuid
        ctypes.CDLL = CDLL

uuid_workaround()

import uuid

def newUUID():
    return str(uuid.uuid1())
