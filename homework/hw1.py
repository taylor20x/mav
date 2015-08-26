# .. -*- coding: utf-8 -*-
#
# ********************************
# hw1.py -- Homework 1 assignment.
# ********************************
# This file must be named hw1.py; none of the function names below are allowed
# to change.
#
# Code skeleton
# =============
# Pick a value from a sequence of items.
def pick_val(
  # The sequence to pick from.
  seq,
  # The element to select. Defaults to 4th from the end. If the sequence is shorter than 4 elements, select the third, second, first, or raise an IndexError exception if the sequence is empty.
  element):

    # Dummy code -- replace this with your code.
    pass

# Tests
# =====
class TestPickVal(object):
    # Test picking from a list with > 4 elements.
    def test_1(self):
        assert pick_val(range(10)) == 6

    # Test picking from a list with exactly 4 elements.
    def test_2(self):
        assert pick_val(range(4)) == 0

    # Test picking from a list with < 4 elements.
    def test_3(self):
        assert pick_val(range(3)) == 0
        assert pick_val(range(2)) == 0
        assert pick_val(range(1)) == 0

    # Test picking from an empty list.
    def test_4(self):
        try:
            pick_val([])
            assert False
        except IndexError:
            pass

# Test driver code
# ================
def run_tests():
    tpv = TestPickVal()
    tpv.test_1()
    tpv.test_2()
    tpv.test_3()
    tpv.test_4()
    print('All tests passed.')

if __name__ == '__main__':
    run_tests()
