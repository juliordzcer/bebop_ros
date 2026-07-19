"""Workaround for a matplotlib/mpl_toolkits version mismatch.

On systems where both a pip-installed matplotlib (e.g. in the user's
`~/.local`) and the distro's `python3-matplotlib` (apt) are present,
Python's import machinery always prefers a *regular* package (one with
an `__init__.py`) over a namespace package portion, regardless of
`sys.path` order. Newer matplotlib versions ship `mpl_toolkits` as a
namespace package (no top-level `__init__.py`), so if an older apt
`python3-matplotlib` is also installed, its `mpl_toolkits/__init__.py`
wins the import unconditionally -- even though `matplotlib` itself
resolved to the newer pip install. That mismatched `mpl_toolkits`
often can't import (e.g. `from matplotlib.tri.triangulation import
Triangulation`, removed in newer matplotlib), which breaks the '3d'
projection with "'3d' is not a valid value for projection".

Because `matplotlib.projections` tries (and, in this situation, fails)
to import and register Axes3D as a side effect of its own module import
-- which happens transitively as soon as anything imports
`matplotlib.figure`/`matplotlib.pyplot`, typically before application
code gets a chance to run -- just making a working `mpl_toolkits.mplot3d`
importable later isn't enough: the '3d' projection name is already
missing from matplotlib's projection registry by then. `ensure_axes3d()`
therefore also explicitly (re-)registers it.

Call `ensure_axes3d()` once, anywhere before the first
`add_subplot(..., projection='3d')` call (import order relative to
matplotlib doesn't matter).
"""

import importlib.util
import os
import sys
import types

import matplotlib
import matplotlib.projections


def ensure_axes3d():
    axes3d_module = sys.modules.get("mpl_toolkits.mplot3d")
    if axes3d_module is None:
        site_packages_dir = os.path.dirname(os.path.dirname(matplotlib.__file__))
        init_path = os.path.join(site_packages_dir, "mpl_toolkits", "mplot3d", "__init__.py")
        if not os.path.isfile(init_path):
            # Not the situation this workaround targets (e.g. a single,
            # consistent matplotlib install) -- fall back to a plain
            # import and let any failure surface normally.
            import mpl_toolkits.mplot3d as axes3d_module
        else:
            if "mpl_toolkits" not in sys.modules:
                package = types.ModuleType("mpl_toolkits")
                package.__path__ = [os.path.join(site_packages_dir, "mpl_toolkits")]
                sys.modules["mpl_toolkits"] = package

            spec = importlib.util.spec_from_file_location(
                "mpl_toolkits.mplot3d", init_path,
                submodule_search_locations=[os.path.dirname(init_path)])
            axes3d_module = importlib.util.module_from_spec(spec)
            sys.modules["mpl_toolkits.mplot3d"] = axes3d_module
            spec.loader.exec_module(axes3d_module)

    if "3d" not in matplotlib.projections.projection_registry.get_projection_names():
        matplotlib.projections.register_projection(axes3d_module.Axes3D)
