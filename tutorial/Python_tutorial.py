# .. -*- coding: utf-8 -*-
#
# ****************************************************
# Python_tutorial.py - A brief introduction to Python.
# ****************************************************
"""
# Running Python
# ==============
# The guaranteed way to make a Python script run: ``python script_name.py``. For
# example, on Windows, ``Python_tutorial.py 1 2`` produces
# ``['C:\\Users\\bjones\\Documents\\mav\\tutorial\\Python_tutorial.py']``.
# In other words, Windows throws the command-line arguments away. To be safe,
# use ``python Python_tutorial.py 1 2`` instead.
from sys import argv
print(argv)
#
# Variables and arithmetic
# ========================
# This program is used to compute compund interest over a given period.
#
# Note that this variable begins as an integer. However, its type dynamicaly
# changes to a float.
principal = 2000
# The amount of money after interest increases the value of the principal.
net_gain = principal
rate = 0.05
numyears = 5
year = 1
while year <= numyears:
    # Before, we used ``principle = principle*(1 + rate)``. Using the ``*=``\
    # operator `DRYs <https://en.wikipedia.org/wiki/Don%27t_repeat_yourself>`_
    # the code. Note that the expression to the right of the ``*=`` operator is
    # implicitly encolosed in parentheses.
    net_gain *= 1 + rate
    # Use the `format <https://docs.python.org/2/library/stdtypes.html#str.format>`_
    # method to nicely format a string.
    print('In year {}, the total value is {}, with an increase of {} from the original investment.\n'.format(year, net_gain, net_gain - principal)),
    # Note that this block isn't placed in a for loop, so we need to manually
    # incrememnt the counter. **So....** we should rewrite this as a for loop.
    #
    # This add 1 to year. **This is bad comment** -- it's doesn't explain why,
    # and it repeats what's already stated in the code.
    year += 1
#
# Conditionals
# ============
a = 5
b = 7
if (a < b and
 number_of_chipmunks < (number_of_squirrels + number_of_rabbits) ):
    pass
elif a > b:
    print('elif')
else:
    print("more")

# Note that ``true`` is the WRONG spelling. Use ``True``.
if true:
    print("True")
else:
    print("False")


if '':
    print("True")
else:
    print("False")
    


# Conversions
# ===========
a = 5
print('The value of a is ' + str(a))
b = '4'
print(a + int(b))


# Strings
# =======
# ...and indexing...
a = 'hello'
print(a[1:2])

print(a[1:5])
print(a[1:])
print(a[-4:])
print("hello "
      "world.")

# Lists
# =====
# There are lots of `list operations <https://docs.python.org/2/library/stdtypes.html#sequence-types-str-unicode-list-tuple-bytearray-buffer-xrange>`_.
l = ['1', 2, 3.5, True]
print(l)

# `Tuples <https://docs.python.org/2/tutorial/datastructures.html#tuples-and-sequences>`_ are unmodifiable lists.
t = (1, '2', True)
# Use an extra comma to create a 1-element tuple.
t1 = (1, )
# Use ``()`` to create an empty tuple.
t2 = ()

# Files
# =====
# Using a `context manager <https://docs.python.org/2/reference/compound_stmts.html#with>`_ here guarantees that the file will be closed. We don't handle Unicode with an simple open command.
with open('../index.rst') as in_file:
    with open('modified_index.rst', 'w') as out_file:
        for line in in_file:
            out_file.write('>>' + line)

# Sets
# ====
# `Sets <https://docs.python.org/2/library/stdtypes.html#set>`_
s = set('hello')
print(s)

# Dictionary
# ==========
# See https://docs.python.org/2/library/stdtypes.html#mapping-types-dict.
d = {'one' : 1, 'two': 2}
print(d['one'])

# Functions
# =========
def foo():
    print('hello')

print(foo())

def bar(a, b, c):
    return b, c, a

#print(bar('1', 2, 3.5))
# Must have the correct number of arguments.
#print(bar(1, 2))
#print(bar(1, 2, 3, 4))

# Pass arg using keyword.
print(bar(None, c=1, b='2'))
"""
"""
def sum(*args):
    total = 0
    for arg in args:
        total += arg
    return total

def summer(num_sum, *args):
    return sum(*args[:num_sum])

#print(sum(3, 4, 5))
#print(summer(2, 3, 4, 5))

# Keyword args
def summer1(*args, **kwargs):
    num_sum = kwargs.get('num_sum', len(args))
    return sum(*args[:num_sum])

print(summer1(3, 4, 5, num_sum=2))
"""
# Testing for data types
#=======================
list = [50, 'fifty']
if isinstance(list[1], int) == False:
	print("wrong data type") 
