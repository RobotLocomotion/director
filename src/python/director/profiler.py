import os
import cProfile
import pstats
import subprocess


class Profiler(object):
    '''
    This class provides some convenience methods to start/stop
    a code profiler and display statistics and a callgraph.
    To generate and view the callgraph you should install gprof2dot
    from the python package index and install xdot:

        pip3 install gprof2dot
        apt-get install xdot

    '''

    def __init__(self):
        self.profiler = None
        self.profile_output = '/tmp/output.{}.profile'.format(os.getpid())
        self.dot_output = '/tmp/callgraph.{}.dot'.format(os.getpid())
        self.enabled = False

    def start(self):
        self.profiler = cProfile.Profile()
        self.profiler.enable()
        self.enabled = True

    def stop(self, show_stats=True, show_callgraph=True):
        self.enabled = False
        self.profiler.disable()
        self.profiler.dump_stats(self.profile_output)

        if show_stats:
            pstats.Stats(self.profiler).sort_stats('cumulative').print_stats()

        if show_callgraph:
            subprocess.check_call(['gprof2dot', '-f', 'pstats', self.profile_output, '-o', self.dot_output])
            subprocess.Popen(['xdot', self.dot_output])
