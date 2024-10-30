"""
This is a script borrowed from Django's source code.
https://github.com/django/django/blob/fc22fdd34f1e55adde161f5f2dca8db90bbfce80/django/utils/decorators.py#L62

The reason for this script is for its last script which is a helper method for creating class method decorators. Used for device decorator which attempts to wrap the __init__ of the class passed in.
More information can be found here: 
https://github.com/django/django/blob/fc22fdd34f1e55adde161f5f2dca8db90bbfce80/docs/topics/class-based-views/intro.txt#L264
"""

from functools import partial, update_wrapper, wraps

from asgiref.sync import iscoroutinefunction, markcoroutinefunction


class classonlymethod(classmethod):
    def __get__(self, instance, cls=None):
        if instance is not None:
            raise AttributeError(
                "This method is available only on the class, not on instances."
            )
        return super().__get__(instance, cls)


def _update_method_wrapper(_wrapper, decorator):
    # _multi_decorate()'s bound_method isn't available in this scope. Cheat by
    # using it on a dummy function.
    @decorator
    def dummy(*args, **kwargs):
        pass

    update_wrapper(_wrapper, dummy)


def _multi_decorate(decorators, method):
    """
    Decorate `method` with one or more function decorators. `decorators` can be
    a single decorator or an iterable of decorators.
    """
    if hasattr(decorators, "__iter__"):
        # Apply a list/tuple of decorators if 'decorators' is one. Decorator
        # functions are applied so that the call order is the same as the
        # order in which they appear in the iterable.
        decorators = decorators[::-1]
    else:
        decorators = [decorators]

    def _wrapper(self, *args, **kwargs):
        # bound_method has the signature that 'decorator' expects i.e. no
        # 'self' argument, but it's a closure over self so it can call
        # 'func'. Also, wrap method.__get__() in a function because new
        # attributes can't be set on bound method objects, only on functions.
        bound_method = wraps(method)(partial(method.__get__(self, type(self))))
        for dec in decorators:
            bound_method = dec(bound_method)
        return bound_method(*args, **kwargs)

    # Copy any attributes that a decorator adds to the function it decorates.
    for dec in decorators:
        _update_method_wrapper(_wrapper, dec)
    # Preserve any existing attributes of 'method', including the name.
    update_wrapper(_wrapper, method)

    if iscoroutinefunction(method):
        markcoroutinefunction(_wrapper)

    return _wrapper


def method_decorator(decorator, name=""):
    """
    Convert a function decorator into a method decorator
    """

    # 'obj' can be a class or a function. If 'obj' is a function at the time it
    # is passed to _dec,  it will eventually be a method of the class it is
    # defined on. If 'obj' is a class, the 'name' is required to be the name
    # of the method that will be decorated.
    def _dec(obj):
        if not isinstance(obj, type):
            return _multi_decorate(decorator, obj)
        if not (name and hasattr(obj, name)):
            raise ValueError(
                "The keyword argument `name` must be the name of a method "
                "of the decorated class: %s. Got '%s' instead." % (obj, name)
            )
        method = getattr(obj, name)
        if not callable(method):
            raise TypeError(
                "Cannot decorate '%s' as it isn't a callable attribute of "
                "%s (%s)." % (name, obj, method)
            )
        _wrapper = _multi_decorate(decorator, method)
        setattr(obj, name, _wrapper)
        return obj

    # Don't worry about making _dec look similar to a list/tuple as it's rather
    # meaningless.
    if not hasattr(decorator, "__iter__"):
        update_wrapper(_dec, decorator)
    # Change the name to aid debugging.
    obj = decorator if hasattr(decorator, "__name__") else decorator.__class__
    _dec.__name__ = "method_decorator(%s)" % obj.__name__
    return _dec


def wraps_method_decorator(decorated_method_name):
    """functools.wraps for class method decorators to copy the metadata of the decorated class method."""

    # Hiro's Opinion: This enables us to do something that is heavily hated by many OOP devs, but is 100% neceesary for our use case.
    # If one of the most popular and "modern" python library is created with this script, I 100% believe is okay to use it.
    # But do not use this if you can not use this decorator on any and all class in your code base.
    # If you want an alternative OOP solution then read up on the decorator pattern. https://en.wikipedia.org/wiki/Decorator_pattern
    def _decorator(func):
        return method_decorator(func, decorated_method_name)

    return _decorator
