import subprocess

_procs = []

def startBotSpy():
    global _procs
    proc = subprocess.Popen(['bot-spy'])
    _procs.append(proc)
    return proc
