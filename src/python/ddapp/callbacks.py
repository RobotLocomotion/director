from weakref import ref
import new

'''
CallbackRegistry is a class taken from matplotlib.cbook.

http://sourceforge.net/p/matplotlib/code/HEAD/tree/trunk/matplotlib/lib/matplotlib/cbook.py
'''

class CallbackRegistry:
    """
    Handle registering and disconnecting for a set of signals and
    callbacks::

       signals = 'eat', 'drink', 'be merry'

       def oneat(x):
           print 'eat', x

       def ondrink(x):
           print 'drink', x

       callbacks = CallbackRegistry(signals)

       ideat = callbacks.connect('eat', oneat)
       iddrink = callbacks.connect('drink', ondrink)

       #tmp = callbacks.connect('drunk', ondrink) # this will raise a ValueError

       callbacks.process('drink', 123)    # will call oneat
       callbacks.process('eat', 456)      # will call ondrink
       callbacks.process('be merry', 456) # nothing will be called
       callbacks.disconnect(ideat)        # disconnect oneat
       callbacks.process('eat', 456)      # nothing will be called

    In practice, one should always disconnect all callbacks when they
    are no longer needed to avoid dangling references (and thus memory
    leaks).  However, real code in matplotlib rarely does so, and due
    to its design, it is rather difficult to place this kind of code.
    To get around this, and prevent this class of memory leaks, we
    instead store weak references to bound methods only, so when the
    destination object needs to die, the CallbackRegistry won't keep
    it alive.  The Python stdlib weakref module can not create weak
    references to bound methods directly, so we need to create a proxy
    object to handle weak references to bound methods (or regular free
    functions).  This technique was shared by Peter Parente on his
    `"Mindtrove" blog
    <http://mindtrove.info/articles/python-weak-references/>`_.
    """

    def __init__(self, signals):
        '*signals* is a sequence of valid signals'
        self.signals = set()
        self.callbacks = dict()
        for s in signals:
            self.addSignal(s)
        self._cid = 0

    def _check_signal(self, s):
        'make sure *s* is a valid signal or raise a ValueError'
        if s not in self.signals:
            signals = list(self.signals)
            signals.sort()
            raise ValueError('Unknown signal "%s"; valid signals are %s'%(s, signals))

    def addSignal(self, sig):
        if sig not in self.signals:
            self.signals.add(sig)
            self.callbacks[sig] = dict()

    def connect(self, s, func):
        """
        register *func* to be called when a signal *s* is generated
        func will be called
        """
        self._check_signal(s)
        proxy = BoundMethodProxy(func)
        for cid, callback in self.callbacks[s].items():
            # Clean out dead references
            if callback.inst is not None and callback.inst() is None:
                del self.callbacks[s][cid]
            elif callback == proxy:
                return cid
        self._cid += 1
        self.callbacks[s][self._cid] = proxy
        return self._cid

    def disconnect(self, cid):
        """
        disconnect the callback registered with callback id *cid*
        """
        for eventname, callbackd in self.callbacks.items():
            try:
                del callbackd[cid]
            except KeyError:
                continue
            else:
                return

    def process(self, s, *args, **kwargs):
        """
        process signal *s*.  All of the functions registered to receive
        callbacks on *s* will be called with *\*args* and *\*\*kwargs*
        """
        self._check_signal(s)
        for cid, proxy in self.callbacks[s].items():
            # Clean out dead references
            if proxy.inst is not None and proxy.inst() is None:
                del self.callbacks[s][cid]
            else:
                proxy(*args, **kwargs)

    def getCallbacks(self, s):
        """
        return callbacks registered to signal *s*.
        """
        self._check_signal(s)
        callbacks = []
        for cid, proxy in self.callbacks[s].items():
            # Clean out dead references
            if proxy.inst is not None and proxy.inst() is None:
                del self.callbacks[s][cid]
            else:
                callbacks.append(proxy)
            return callbacks


class BoundMethodProxy(object):
    '''
    Our own proxy object which enables weak references to bound and unbound
    methods and arbitrary callables. Pulls information about the function,
    class, and instance out of a bound method. Stores a weak reference to the
    instance to support garbage collection.

    @organization: IBM Corporation
    @copyright: Copyright (c) 2005, 2006 IBM Corporation
    @license: The BSD License

    Minor bugfixes by Michael Droettboom
    '''
    def __init__(self, cb):
        try:
            try:
                self.inst = ref(cb.im_self)
            except TypeError:
                self.inst = None
            self.func = cb.im_func
            self.klass = cb.im_class
        except AttributeError:
            self.inst = None
            self.func = cb
            self.klass = None

    def __call__(self, *args, **kwargs):
        '''
        Proxy for a call to the weak referenced object. Take
        arbitrary params to pass to the callable.

        Raises `ReferenceError`: When the weak reference refers to
        a dead object
        '''
        if self.inst is not None and self.inst() is None:
            raise ReferenceError
        elif self.inst is not None:
            # build a new instance method with a strong reference to the instance
            mtd = new.instancemethod(self.func, self.inst(), self.klass)
        else:
            # not a bound method, just return the func
            mtd = self.func
        # invoke the callable and return the result
        return mtd(*args, **kwargs)

    def __eq__(self, other):
        '''
        Compare the held function and instance with that held by
        another proxy.
        '''
        try:
            if self.inst is None:
                return self.func == other.func and other.inst is None
            else:
                return self.func == other.func and self.inst() == other.inst()
        except Exception:
            return False

    def __ne__(self, other):
        '''
        Inverse of __eq__.
        '''
        return not self.__eq__(other)
