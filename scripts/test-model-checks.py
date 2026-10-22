#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Export checks tolerate libm rounding but reject optical/provenance changes."""
import copy
import json
import math
from pathlib import Path
import unittest

from model_checks import compare_header, compare_model

ROOT = Path(__file__).resolve().parents[1]


class ExportChecks(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((ROOT / 'integrations/python/mid360_sim/nominal.json').read_text())
        self.header = (ROOT / 'integrations/gazebo/include/mid360/model.hh').read_text()

    def test_one_ulp_rays_pass_without_rewriting_model(self):
        model = copy.deepcopy(self.model)
        for ray in model['reference_rays']:
            ray['direction'] = [math.nextafter(x, math.inf) for x in ray['direction']]
        compare_model(self.model, model)
        value = self.model['reference_rays'][0]['direction'][0]
        header = self.header.replace(str(value), str(math.nextafter(value, math.inf)))
        self.assertNotEqual(header, self.header)
        compare_header(self.header, header)

    def test_coefficients_clocks_indices_and_real_direction_changes_fail(self):
        for change in ('coefficient', 'clock', 'index', 'direction'):
            model = copy.deepcopy(self.model)
            if change == 'coefficient':
                model['coefficients'][0][0][0] += 1
            elif change == 'clock':
                model['spin_hz'] += 1
            elif change == 'index':
                model['reference_rays'][0]['index'] += 1
            else:
                model['reference_rays'][0]['direction'][0] += 1e-10
            with self.subTest(change=change), self.assertRaises(AssertionError):
                compare_model(self.model, model)

    def test_cpp_constants_notices_and_fixtures_are_checked(self):
        value = str(self.model['reference_rays'][0]['direction'][0])
        for changed in (
            self.header.replace('SpinHz=181', 'SpinHz=182'),
            self.header.replace('SPDX-License-Identifier', 'license-missing'),
            self.header.replace(value, '-0.5'),
            self.header.replace('{0, {', '{9, {', 1),
        ):
            self.assertNotEqual(changed, self.header)
            with self.assertRaises(AssertionError):
                compare_header(self.header, changed)


if __name__ == '__main__':
    unittest.main()
