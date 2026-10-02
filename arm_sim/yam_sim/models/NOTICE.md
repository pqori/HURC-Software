# Third-party models

Everything in this `models/` directory except this file is copied **unmodified** from
[i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt) at commit
`120c3c81400171174604e503943f8d1ebc891058` (committed 2026-09-17).

| Here | Upstream path |
| --- | --- |
| `arm/yam/v1/` (`yam.xml`, `yam.urdf`, `README.md`, `assets/*.stl`) | `i2rt/robot_models/arm/yam/v1/` (the `assets/crank_4310/` duplicate, used only by upstream station models, was left out) |
| `gripper/linear_4310/`, `gripper/crank_4310/`, `gripper/no_gripper/` | `i2rt/robot_models/gripper/<name>/` |
| `config/yam_v1.yml`, `config/{linear_4310,crank_4310,no_gripper}.yml` | `i2rt/robots/config/` |

`yam_sim/assembly.py` reads these files and builds the simulation scene at runtime. It does not
edit them on disk.

The upstream code is MIT licensed. The license text follows and is also in `LICENSE.i2rt`.

```
MIT License

Copyright (c) I2RT Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

Some code in `yam_sim/` is adapted from the same upstream commit: the arm and gripper merge in
`assembly.py` (from `i2rt/robots/utils.py::combine_arm_and_gripper_xml`), the gripper force
limiter in `motor.py` (from `GripperForceLimiter`), and the command and observation conventions in
`robot.py` (from `MotorChainRobot` and `SimRobot`). Those parts are under the same MIT license.
