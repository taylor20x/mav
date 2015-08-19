# .. -*- coding: utf-8 -*-
#
# ****************************************************
# Python_tutorial.py - A brief introduction to Python.
# ****************************************************
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
# Note that this begins as an integer. However, the type of ``principal``
# dynamicaly changes to a float.
principal = 1000
rate = 0.05
numyears = 5
year = 1
while year <= numyears:
    # Before, we used ``principle = principle*(1 + rate)``. Using the ``*=``\
    # operator `DRYs <https://en.wikipedia.org/wiki/Don%27t_repeat_yourself>`_
    # the code. Note that the expression to the right of the ``*=`` operator is
    # implicitly encolosed in parentheses.
    principal *= 1 + rate
    print('In year {}, the total value is {}, with an increase of {} from the original investment.\n'.format(year, principal, principal - 1000)),
    # Note that this block isn't placed in a for loop, so we need to manually
    # incrememnt the counter. **So....** we should rewrite this as a for loop.
    #
    # This add 1 to year. **This is bad comment** -- it's doesn't explain why,
    # and it repeats what's already stated in the code.
    year = year + 1

