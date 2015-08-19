# .. -*- coding: utf-8 -*-
#
# Intro to Python, part 1
# =======================
# By Bryan A. Jones
#
# Calculator
# ----------
# Python, used from the interactive prompt, makes a nice
# calculator. Copy and paste the following examples to try
# it out.
#
# >>> 100 + 3.5
# 103.5
# >>> 'hello ' + "world"
# 'hello world'
# >>> True or False
# True
#
# `Common data types
# <http://docs.python.org/library/stdtypes.html#numeric-types-int-float-long-complex>`_,
# as shown above, include integers, floating-point numbers,
# strings, and Boolean values. Python provides the usual
# assortment of operators as stated in the table at the end
# of the previous link, plus `Boolean operators
# <http://docs.python.org/library/stdtypes.html#boolean-operations-and-or-not>`_
# and `comparisons
# <http://docs.python.org/library/stdtypes.html#comparisons>`_.
#
# Exercises for strings:
#
# #. Is ``'hello'`` the same as ``"hello"``? To check,
#    compare them with ``==``. Explain.
#
#    I ran the statement ``your code here``, which produced
#    the output ``your output here``. This means that...
# #. Per the `strings
#    <http://docs.python.org/tutorial/introduction.html#strings>`_
#    topic, what is the third way to create a string? Give
#    an example.
# #. What is a raw string? How would you enter the string
#    ``l\m\n``?
# #. What does multiplying a string do?
# #. Can strings be divided?
#
# Exercises for numbers: first, execute the statement below
# before working these exercises.
from __future__ import division
#
# #. Based on the links above, give examples of all the
#    usual arithmetic operations (addition, subtraction,
#    multiplication, division).
#
#    | Addition: ``your code here``.
#    | Subtraction: ``your code here``.
#    | Multiplications:  ``your code here``.
#    | Division:  ``your code here``.
# #. What is ``1//2``? What about ``1/2``? Why are they
#    different?
#
#    Evaluating ``1//2`` produces ``your result here``.
#    Evaluating ``1/2`` produces ``your result here``.
#    These are different because **your explanation here**.
# #. Based on the links above, give an example of finding
#    :math:`|x|`, the absolute value of ``x``.
#
#    Absolute value: ``your code here``.
# #. Based on the links above, give an example of finding
#    :math:`x^y` (``x`` to the ``y``\ th power).
#
#    Exponentition: ``your code here``.
#
# Variables and assignments
# -------------------------
# The ``=`` operator assigns a value, rather than comparing
# it.
#
# >>> a = 5
# >>> a
# 5
# >>> b = a
# >>> b
# 5
#
# Exercises for comparisons:
#
# #. Assign a value to ``a`` and ``b``. How do you ask
#    Python if ``a`` is less than ``b``? Repeat for other
#    comparisons.
#
#    | Less than: ``your code here``.
#    | Greater than: ``your code here``.
#    | Less than or equal; ``your code here``.
#    | Greater than or equal: ``your code here``.
#    | Equality:  ``your code here``.
#    | Inequality: ``your code here``.
#
# #. What does ``5 <= a < 20`` mean? Is it true or false?
#
#    ``x`` produces  ``your code here`` because **your
#    explanation here.**
#
# Conditionals
# ------------
# Python also provdes ``if`` statements.
a = 3
b = 4
if a < b:
    print('a is less.')
elif a > b:
    print('b is less.')
else:
    print('a and b are equal.')

# Exercises:
#
# #. What does the ``print`` statement do? **Your
#    explanation here**.
# #. Can ``print`` be applied to variables? To expressions
#    such as ``a + 5``? Give examples.
#
#    | Print variables: ``your code here``.
#    | Print expressions: ``your code here``.
# #. What does ``elif`` mean? **Your explanation here**.
# #. What does Python consider to be true? Find out, by
#    converting an object to a Boolean via the `bool
#    <http://docs.python.org/library/functions.html#bool>`_
#    operator. Give three examples of true items and three
#    examples of false items, looking at strings, integers,
#    and floats.
#
#    | False string: ``your code here``.
#    | True string: ``your code here``. What other strings
#      are ``True``?
#    | False integer: ``your code here``.
#    | True integer: ``your code here``. What other integers
#      are ``True``?
#    | False float: ``your code here``.
#    | True float: ``your code here``. What other floats are
#      ``True``?
# #. Write a script which prints "b is between a and c" when ``a > b > c``.

