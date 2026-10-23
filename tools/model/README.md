# The model-building scripts

The one-off scripts that made the cart model (`scripts/model/cart/`) and fitted the arm's collision capsules
(`scripts/model/utra6_1000/collision.json`), kept so both can be made again. They're as they ran on this PC on
2026-10-20, so the paths at the top of the `sw_export` scripts (`ASM`, the cart assembly in a clone of
[Inventor22/umbratek-cad](https://github.com/Inventor22/umbratek-cad), and `SWCONST`, SolidWorks' constants) are this
PC's: change them for another. Nothing else in the repo runs them.

## The cart

From the SolidWorks assembly `Utra 1000/UTRA 1000 Aluminum Extrusion Assembly.SLDASM` in umbratek-cad, in three
steps into one export folder, from the repo's root:

```
python tools/model/sw_export2.py EXPORT        # with SolidWorks 2026 running; pywin32
python tools/model/sw_export3.py EXPORT        # the same
python tools/model/build_cart2.py EXPORT .     # NumPy
```

1. `sw_export2.py` attaches to SolidWorks, opens the assembly read-only and silently, and writes `components.json`
   (every component's file and placement), each unique part as an STL in its own frame (`parts/`), and the whole
   assembly as one STL in assembly coordinates, as a cross-check. It saves nothing back to the CAD.
2. `sw_export3.py`, a second pass, writes each component's bounding box (`boxes.json`), each component as an STL in
   assembly coordinates (`components/`), and the arm's `1000-BASE` and `1000-Link1` parts in their own frames, which
   line the cart up with Umbratek's URDF.
3. `build_cart2.py` writes `scripts/model/cart/utra_cart.urdf` and its display meshes,
   `meshes/{frame,casters,controller}.STL`, in the arm's base frame (its usage line calls it `build_cart.py`).
   `scripts/model/cart/SOURCE.txt` describes the result.

The export isn't kept: it's 631 files, 1.1 GB.

`sw_export.py` and `build_cart.py` are the first drafts that `sw_export2.py` and `build_cart2.py` replaced. The first
didn't allow for pywin32 returning a zero-argument COM method's value on attribute access; the second made one display
mesh for the whole cart.

## The arm's collision capsules

```
python tools/model/fit_capsules.py scripts/model/utra6_1000/meshes capsules.json     # NumPy
```

`fit_capsules.py` fits capsules to Umbratek's link meshes, one per housing or tube along lines from the joint axes,
and a cylinder to the base, and writes them as `capsules` and `base`, as in `collision.json`. That file's `skip` pairs
and `about` were added afterwards; `scripts/model/utra6_1000/SOURCE.txt` says how both were chosen.

## Checked

Run again on 2026-10-23: `build_cart2.py` on the original export wrote `utra_cart.urdf` and the three meshes byte for
byte as committed, and `fit_capsules.py` on the repo's meshes gave exactly the committed capsules and base.
