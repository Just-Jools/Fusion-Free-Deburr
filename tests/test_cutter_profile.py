import copy
import unittest

from FusionFreeDeburr import cutter_profile as cp
from FusionFreeDeburr import deburr_math as dm


class CutterProfileTests(unittest.TestCase):
    def setUp(self):
        self.raw = {'type': 'lollipop mill', 'unit': 'millimeters',
                    'geometry': {'DC': 8, 'RE': 4, 'shoulder-diameter': 4, 'LCF': 8,
                                 'shoulder-length': 20, 'OAL': 75, 'SFDM': 6}}

    def test_document_tool_dimensions(self):
        cutter = cp.from_tool_json(self.raw)
        self.assertAlmostEqual(cutter.radius, .4)
        self.assertAlmostEqual(cutter.max_contact_height, .3464101615)

    def test_inch_and_mm_tools_match(self):
        inches = copy.deepcopy(self.raw)
        inches['unit'] = 'inches'
        inches['geometry'] = {k: v / 25.4 for k, v in inches['geometry'].items()}
        a, b = cp.from_tool_json(self.raw), cp.from_tool_json(inches)
        self.assertAlmostEqual(a.radius, b.radius)
        self.assertAlmostEqual(a.max_contact_height, b.max_contact_height)

    def test_ball_retains_hemisphere_limit(self):
        self.raw['type'] = 'ball end mill'
        self.assertEqual(cp.from_tool_json(self.raw).max_contact_height, 0)

    def test_bad_profiles_are_rejected(self):
        for key, value in (('RE', 3), ('DC', 0), ('shoulder-diameter', 8),
                           ('LCF', float('nan'))):
            raw = copy.deepcopy(self.raw)
            raw['geometry'][key] = value
            with self.subTest(key=key), self.assertRaises(dm.DeburrError):
                cp.from_tool_json(raw)
        self.raw['unit'] = 'unknown'
        with self.assertRaises(dm.DeburrError):
            cp.from_tool_json(self.raw)

    def test_shaft_and_holder_are_included(self):
        self.raw['shaft'] = {'segments': [{'height': 50, 'lower-diameter': 8,
                                          'upper-diameter': 12}]}
        self.raw['geometry']['assemblyGaugeLength'] = 80
        self.raw['holder'] = {'unit': 'inches', 'gaugeLength': 1,
                              'segments': [{'height': 1, 'lower-diameter': 1,
                                            'upper-diameter': 2}]}
        sections = cp.from_tool_json(self.raw).noncutting_sections
        self.assertAlmostEqual(sections[2][0], 2)
        self.assertAlmostEqual(sections[2][1], 7)
        self.assertAlmostEqual(sections[2][2], .6)
        self.assertAlmostEqual(sections[-1][0], 5.46)
        self.assertAlmostEqual(sections[-1][2], 2.54)

    def test_short_flutes_include_noncutting_spherical_cap(self):
        self.raw['geometry']['LCF'] = 7
        cutter = cp.from_tool_json(self.raw)
        self.assertAlmostEqual(cutter.max_contact_height, .3)
        self.assertTrue(any(abs(bottom - .7) < 1e-9 for bottom, _, _ in cutter.noncutting_sections))

    def test_bad_shaft_segment_is_not_ignored(self):
        self.raw['shaft'] = {'segments': [{'height': -1, 'lower-diameter': 8,
                                          'upper-diameter': 8}]}
        with self.assertRaisesRegex(dm.DeburrError, 'shaft or holder'):
            cp.from_tool_json(self.raw)
