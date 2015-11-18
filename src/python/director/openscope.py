import subprocess

_procs = []

def startSignalScope():
    global _procs
    proc = subprocess.Popen(['signal-scope'])
    _procs.append(proc)
    return proc
