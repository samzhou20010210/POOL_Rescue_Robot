# Vendored GCOPTER headers

Source:   https://github.com/ZJU-FAST-Lab/GCOPTER
Revision: e0444f6d47b84f972ced91746b05feb36ce1fd4f (main branch; the commit
          recorded in the GitHub archive GCOPTER-main.zip)
Files:    include/gcopter/*.hpp, copied UNMODIFIED from gcopter/include/gcopter/.

Only the optimizer and corridor pieces are vendored (implementation step 6).
Not vendored: the ROS 1 code, OMPL, flatness.hpp, sfc_gen.hpp, gcopter.hpp,
voxel_map.hpp, voxel_dilater.hpp.

| File | Purpose | Needed by | License |
|---|---|---|---|
| minco.hpp | MINCO trajectory class | step 7 | MIT (GCOPTER, see LICENSE) |
| trajectory.hpp | piecewise polynomials | minco.hpp | MIT (GCOPTER) |
| root_finder.hpp | polynomial roots | trajectory.hpp | MIT (GCOPTER), file header: Copyright (c) 2020 Zhepei Wang |
| lbfgs.hpp | L-BFGS optimizer | firi.hpp, step 7 | MIT (GCOPTER LICENSE; the file has no own header) |
| firi.hpp | FIRI convex inflation | corridor_2d | MIT (GCOPTER) |
| sdlp.hpp | Seidel's small LP | firi.hpp, geo_utils.hpp | own notice in the file (Hohmeyer 1990, Wang 2021): redistribution allowed if the notice is kept |
| geo_utils.hpp | polytope utilities | step 7 | MIT (GCOPTER) |
| quickhull.hpp | 3D convex hull | geo_utils.hpp | public domain (Antti Kuukka), stated in the file |

LICENSE in this folder is GCOPTER's MIT license, copied unchanged. The
per-file notices inside sdlp.hpp and quickhull.hpp are kept as they are.

These files are excluded from the ament linters (third-party style).

SHA-256 of the vendored files (to check they are unmodified):

```
e47441312b31dff209a59798a8414cbdf310534720e984c73894cb7ea1beb6ee  include/gcopter/firi.hpp
9a1e9b70c9a15ef46c2f8658df293f29051e98ab7dac5ffad129bfa8b9d87031  include/gcopter/geo_utils.hpp
fdc6b0d4c4b4c50e0368d6983dd28ffd893c824df461aebe9e3b24dd8a99b3ca  include/gcopter/lbfgs.hpp
60defee515dd3c96cdc54f3dbfa279b84556c92fce92b77365cc37b70649b512  include/gcopter/minco.hpp
c0d2668c2c87ff9dfddc750d2bca9f91e4846f8fa78beffe44def0736b5e0e21  include/gcopter/quickhull.hpp
c5eaa3ebed6bc65b5671db8965c0c6e3f79cb0cfa9787c89822315cdbac9d781  include/gcopter/root_finder.hpp
d7b7bd66597ca1355d7bf16d902cf17da638ad0fd98870673f209a329ded0abf  include/gcopter/sdlp.hpp
9ba9b8d43be34848f9f3bd955d5bf8c8922e85757255a7363c82fe59a8a2e9fd  include/gcopter/trajectory.hpp
```
