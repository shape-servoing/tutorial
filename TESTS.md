# Tests

```bash
python run.py --tests              # creates .venv on first use, installs the requirements, runs the tests
python -m unittest discover tests  # the same, in an environment with numpy
```

`tests/test_web_parity.py` also needs Node.js; without it, those tests are skipped.

## The numbers of the tutorial

`tests/test_tutorial_numbers.py` checks the values quoted in the tutorial and its supplementary material:

- Sections II-IV (level 1): the offline residual (0.31, about 18% of 1.72), the singular values (2.54, 1.58, 0.15) and condition number (about 17) of the estimate, the closed-loop behaviour with and without the online update and with a large gain, the reachable and synthetic infeasible targets, and the range/null-space decompositions.
- Section VI, feature choice: after a disturbance during the first 20 of 80 iterations, point coordinates return the rod to the target pose, while edge vectors leave it shifted and curvature-based features shifted and rotated, all below 3% of the initial feature error; on the looped rod, the peak axial strain is about 27% with point coordinates, 3% with edge vectors, and at most the initial 2.1% with curvature-based features.
- Section VII, failure modes: sigma_min 0.15 to 0.04, cond 16 to 42, w by a factor of 21, and wrist share 68% to 78% on the taut rod; buckling onset at 5.8% compression.
- Appendix E, extensions: moving-target errors with and without feedforward (below 1.7e-3, peaks of 0.46); the secondary task of Fig. S3 with and without null-space projection (shape error below 1.4e-2 rad and up to 0.65 rad; with the demo defaults, below 1.2e-3 rad and 0.14 rad); the combined curvature and point-coordinate task of Fig. S4 (without transport, a point-coordinate error of 3.6; with it, below 1e-4, curvature error below 1e-8, and the axial strain never above its initial value, with no pause of 20 iterations or more before the transport); and the 3-D shape control runs (below 1% within 20 iterations).

`tests/test_subspaces_demo.py` and `tests/test_spatial_demo.py` check the range/null-space demo and the 3-D demo.

## The demo pages

- `tests/test_web_parity.py` runs scripted actions on every demo in both implementations, the JavaScript sessions of the pages (through `tests/web_parity.mjs`, with Node.js) and the Python sessions in `demos/`, and compares the results: single evaluations agree to 1e-6 (relative), and closed-loop runs of up to 300 iterations to 1e-4. Not compared are the quantities that depend on random numbers (random targets, probe noise, and random perturbations, since the two implementations use different generators) and the directions whose sign or basis the singular value decomposition leaves free; in the pages, the singular triplets give each feature direction phi_l its largest entry positive.
- `tests/test_web_build.py` checks that the pages are self-contained and match their sources (see [DEVELOPMENT.md](DEVELOPMENT.md)).
- `tests/test_web_pages.py` checks their content and links.
