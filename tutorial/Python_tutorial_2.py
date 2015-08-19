# .. -*- coding: utf-8 -*-
#
# Intro to Python, part 2
# =======================
# By Bryan A. Jones
#
# Functions
# ---------
# It's easy to write short functions. Let's approximate the
# value of :math:`e` using :math:`e = \sum \limits_{n = 0}^\infty  {\frac{1}{{n!}}}`.
# To do so, let's make use of the `math
# <http://docs.python.org/library/math.html>`_ library.
import math
# For loops operate on a list. Use `range <http://docs.python.org/library/functions.html#range>`_ to create a list of numbers:
sum = 0
for n in range(5):
    sum += 1.0/math.factorial(n)

# In Python, indents group statements together, while a colon indicates the start of a sub-group of statements. Don't use tabs; generally 4 spaces for an indent works well. `Computing the Fibonnaci sequence <http://docs.python.org/tutorial/introduction.html#first-steps-towards-programming>`_ makes for another good programming example.
#
# To format the output nicely, Python provides the `modulo (%) operator for strings <http://docs.python.org/library/stdtypes.html#string-formatting>`_.
print('Value of e: summation is %1.3f; value of e is %1.3f; difference is %1.3f.' % (sum, math.e, sum - math.e))

# Exercises:
#
# #. Try ``from math import factorial`` on one line, then ``factorial(3)`` on another. Explain there difference between ``import math`` and ``from math import factorial``.
# #. Write a program to compute :math:`n!`.
#
# Sequences
# ---------
# Python provides several `sequence <http://docs.python.org/library/stdtypes.html#sequence-types-str-unicode-list-tuple-bytearray-buffer-xrange>`_ types, in addition to strings:
#
# A list is a `mutable <http://docs.python.org/library/stdtypes.html#typesseq-mutable>`_ sequence:
point = [3, 4]
# A tuple is an immutable sequence:
fixed_point = (5, 6, 7)
# String are also immutable and provide many additional `methods <http://docs.python.org/library/stdtypes.html#string-methods>`_ and `operations <http://docs.python.org/library/string.html>`_.
string = 'hello'
#
# Indexing starts with 0, as in C.
#
# >>> point[0]
# 3
#
# >>> fixed_point[1]
# 6
#
# Negative indexes count from the last element.
#
# >>> string[-1]
# 'o'
#
# Indexing ends with the previous element. A helpful way to think: the indices live bewtween elements:
##  +---+---+---+---+---+
##  | h | e | l | l | o |
##  +---+---+---+---+---+
##  0   1   2   3   4   5
## -5  -4  -3  -2  -1
#
# >>> string[0:2]
# 'he'
#
# You may omit the beginning or end of a sequence.
#
# >>> string[2:]
# 'llo'
# >>> fixed_point[:2]
# (5, 6)
#
# Exercises:
#
# #. What sequences are true? false? (Use the ``bool()`` operator to test them).
# #. How do you capitalize the last *n* characters of ``string``?
# #. Create a list containing ``point``, your name, and a random floating-point number. How can you refer to element 1 of point inside this list?
# #. What does ``string[0:5:2]`` do? Explain.
# #. Make a list which contains ``point`` twice. Set ``point[0] = 'hi'``. What happened to this list? Why?
#
# Dictionaries
# ------------
# A `dictionary <http://docs.python.org/library/stdtypes.html#typesmapping>`_ associates an key (which much be immutable, such as a number, string, or tuple) with a value.
#
# >>> d = {'Isaac' : 'Newton', 'F' : 'ma', 9.8 : 'm/s^2'}
# >>> d['F']
# 'ma'
#
# Exercises:
#
# #. Create a dictionary which maps 'first' to your first name, 'last' to your last name, and 'middle' to your middle name.
# #. Create a dictionary which maps your last name to a second dictionary which maps your first name to your age, plus another random first name to another age. What syntax retrieves your age?
# #. Use the ``dict()`` operator to convert a list or typle to a dictionary. How are key and value assigned?
# #. Use the ``list()`` operator to convert a dictionary to a list. How do list elements correspond to keys and values?
# #. What happens if you add the same key twice to a dictionary?
#
# Functions
# ---------
# The `def keyword <http://docs.python.org/tutorial/controlflow.html#defining-functions>`_ creates a function; use the return statement to return values.
def add_two(num):
    return num + 2

# >>> add_two(3)
# 5

# Functions allow default arguments; return with commas returns a tuple.
def swap(first, second = 'bear'):
    return second, first

# >>> a, b = swap(1)
# >>> a, b
# ('bear', 1)

# Exercises:
#
# #. Write a function which, when given a vector ``v`` (as a list or tuple), returns its length.
# #. Write a function which, when given vectors ``v1`` and ``v2``, returns the distance between them.
#
# Classes
# -------
# Classes provide a convenient way to collect methods (the name for functions inside a class) together with the data they operate on. The parameter to a gives the object(s) from which it inhereits; by default, use ``object``, the parent of all objects in Python.
class MyClass(object):
    # The ``__init__`` method is called when the object is created; this is the classes' constructor. The first parameter to any method is the object itself, traditionally named ``self``.
    def __init__(self, num):
        # The object can store anything inside it using ``self``.
        self.my_num = num
        self.apples = 'red'
        print('Ready.')
    def inventory(self):
        print('%d %s apples' % (self.my_num, self.apples))

# Create an instance of a class by invoking it, passing parameters to ``__init__``.
#
# >>> my_inst = MyClass(3)
# Ready.
#
# Invoke methods on a class using the dot operator:
#
# >>> my_inst.inventory()
# 3 red apples

# Exercises:
#
# #. Create a class named ``vector`` that accepts a list in its constructor, then provides ``length``, ``add(v2)``, and ``sub(v2)`` methods.
#
# Unit testing
# ------------
#
# .. _run_tests:
#
# To run this, you must first install `Py.Test <http://pytest.org/>`_. At the command prompt, type ``C:\Python27\Scripts\pip install pytest``.
import pytest
def run_tests():
    # `Run all tests <http://pytest.org/latest/usage.html#calling-pytest-from-python-code>`_. If your test code was in a file named ``*_test.py``, you don't need to give its name below -- pytest will find it automatically.
    pytest.main('Python_tutorial.py')
    # Run a specifically-named test -- see above link and `this <http://pytest.org/latest/usage.html#specifying-tests-selecting-tests>`_.
    pytest.main('Python_tutorial.py -k test_swap_default')

# Here's an example of a test which pytest will find (since the name of this function begins with ``test_``) and run.
def test_swap_default():
    a, b = swap(1)
    assert a == 'bear'
    assert b == 1

# Here's another test
def test_swap_both():
    a, b = swap(1, 2)
    assert (a, b) == (2, 1)

# An example of a test failure
def test_fail():
    a, b = swap(1, '2')
    assert (a, b) == (2, 1)


# To run the test, call run_tests_.
run_tests()

# Further reading
# ---------------
# Read the `Python documentation <http://docs.python.org/index.html>`_ for a deeper coverage of the language.
#
# A bit of exploration into how Python variables work:

from collections import Sequence
def print_id(obj, indent = 0):
    print('%s%s at 0x%08x' % (' '*indent, type(obj).__name__, id(obj))),
    if isinstance(obj, Sequence) and not isinstance(obj, str):
        for element in obj:
            print('\n'),
            print_id(element, indent + 2)
    else:
        print('= ' + str(obj)),
    if indent == 0:
        print('\n'),

# Python stores all its variables in dictionaries. In this instance, ``point`` refers to a list at memory location 0x02a62080.
t = (point, point)
# >>> print_id(t)
# tuple at 0x02bd4350
#   list at 0x02bd2080
#     int at 0x01e55d70 = 3
#     int at 0x01e55d64 = 4
#   list at 0x02bd2080
#     int at 0x01e55d70 = 3
#     int at 0x01e55d64 = 4
#
# Changing an element of ``point`` changes both, because ``point`` refers to one object, not two identical objects.
t[0][0] = 'changed'
# >>> print_id(t)
# tuple at 0x02bd4350
#   list at 0x02bd2080
#     str at 0x01f63ce0 = changed
#     int at 0x01e55d64 = 4
#   list at 0x02bd2080
#     str at 0x01f63ce0 = changed
#     int at 0x01e55d64 = 4

a = 4
# >>> print_id(a)
# int at 0x01e55d64 = 4
#
# Since Python already has the integer object 4 handy at address 0x01e85d64, ``a`` now refers to it as well as the second element of the list.
