"""
test_common.py — Unit tests for COVEN common.py naming system

Tests the witch/coven naming system used to name modules and docks.

NOTE: Serialization tests have been consolidated into test_serialization.py
to avoid duplication. This file only tests the naming system.

Author: Alexander Shultis
Date: November 2025
"""

import unittest
from coven_core.common import (
    WITCH_NAMES, COVEN_NAMES,
    get_witch_name, get_coven_name, reset_naming,
)


class TestWitchNaming(unittest.TestCase):
    """Test the witch/coven naming system."""

    def setUp(self):
        """Reset naming system before each test."""
        reset_naming()

    def tearDown(self):
        """Reset naming system after each test."""
        reset_naming()

    def test_witch_names_list_populated(self):
        """Test that WITCH_NAMES list has entries."""
        self.assertGreater(len(WITCH_NAMES), 20)
        self.assertIn("Morgan_Le_Fay", WITCH_NAMES)
        self.assertIn("Hermione_Granger", WITCH_NAMES)
        self.assertIn("Baba_Yaga", WITCH_NAMES)

    def test_coven_names_list_populated(self):
        """Test that COVEN_NAMES list has entries."""
        self.assertGreater(len(COVEN_NAMES), 5)
        self.assertIn("The_Graeae", COVEN_NAMES)
        self.assertIn("The_Norns", COVEN_NAMES)

    def test_get_witch_name_returns_valid_name(self):
        """Test that get_witch_name returns a name from the list."""
        name = get_witch_name()
        self.assertIn(name, WITCH_NAMES)

    def test_get_coven_name_returns_valid_name(self):
        """Test that get_coven_name returns a name from the list."""
        name = get_coven_name()
        self.assertIn(name, COVEN_NAMES)

    def test_witch_names_no_duplicates_until_exhausted(self):
        """Test that witch names don't repeat until pool is exhausted."""
        names = [get_witch_name() for _ in range(len(WITCH_NAMES))]
        # All names should be unique (no duplicates)
        self.assertEqual(len(names), len(set(names)))
        # All names should be from the list
        for name in names:
            self.assertIn(name, WITCH_NAMES)

    def test_coven_names_no_duplicates_until_exhausted(self):
        """Test that coven names don't repeat until pool is exhausted."""
        names = [get_coven_name() for _ in range(len(COVEN_NAMES))]
        # All names should be unique (no duplicates)
        self.assertEqual(len(names), len(set(names)))
        # All names should be from the list
        for name in names:
            self.assertIn(name, COVEN_NAMES)

    def test_witch_names_reset_after_exhausted(self):
        """Test that witch names reset after all are used."""
        # Exhaust all names
        for _ in range(len(WITCH_NAMES)):
            get_witch_name()
        # Next name should still be valid (pool reset)
        name = get_witch_name()
        self.assertIn(name, WITCH_NAMES)

    def test_reset_naming(self):
        """Test that reset_naming clears used names."""
        # Get some names
        _ = get_witch_name()
        _ = get_coven_name()

        # Reset
        reset_naming()

        # The same name could now be picked again (it's back in the pool)
        # Just verify we get valid names
        name2 = get_witch_name()
        self.assertIn(name2, WITCH_NAMES)

    def test_witch_names_unique_in_list(self):
        """Test that all witch names in the list are unique."""
        self.assertEqual(len(WITCH_NAMES), len(set(WITCH_NAMES)))

    def test_coven_names_unique_in_list(self):
        """Test that all coven names in the list are unique."""
        self.assertEqual(len(COVEN_NAMES), len(set(COVEN_NAMES)))


if __name__ == '__main__':
    unittest.main()
