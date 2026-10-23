"""Second pass: per-component bounding boxes and per-component STLs (assembly coordinates), plus the arm's
base part in its own frame. Read-only: nothing is saved back to the CAD files."""
import glob
import json
import os
import sys
import time

import pythoncom
import win32com.client
from win32com.client import VARIANT

ASM = r"C:\Users\Dustin\Documents\Github\umbratek-cad\Utra 1000\UTRA 1000 Aluminum Extrusion Assembly.SLDASM"
SWCONST = r"C:\Program Files\Dassault Systemes\SOLIDWORKS 3DEXPERIENCE R2026x\SOLIDWORKS\swconst.tlb"
OUT = sys.argv[1]


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def call(obj, name, *args):
    member = getattr(obj, name)
    if isinstance(member, win32com.client.dynamic.CDispatch):
        return member
    return member(*args) if callable(member) else member


def constants(path):
    lib = pythoncom.LoadTypeLib(path)
    out = {}
    for i in range(lib.GetTypeInfoCount()):
        info = lib.GetTypeInfo(i)
        attr = info.GetTypeAttr()
        if attr.typekind == pythoncom.TKIND_ENUM:
            for j in range(attr.cVars):
                desc = info.GetVarDesc(j)
                out[info.GetNames(desc.memid)[0]] = desc.value
    return out


def byref_int():
    return VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)


def save_as(sw, doc, path):
    sw.ActivateDoc3(call(doc, "GetTitle"), False, 0, byref_int())  # SolidWorks exports the active document
    errors, warnings = byref_int(), byref_int()
    ok = call(doc, "Extension").SaveAs(path, 0, 1, VARIANT(pythoncom.VT_DISPATCH, None), errors, warnings)
    return ok, errors.value, warnings.value


def main():
    C = constants(SWCONST)
    sw = win32com.client.GetActiveObject("SldWorks.Application")
    log(f"Attached to SolidWorks {call(sw, 'RevisionNumber')}")
    sw.SetUserPreferenceToggle(C["swSTLBinaryFormat"], True)
    sw.SetUserPreferenceToggle(C["swSTLDontTranslateToPositive"], True)
    sw.SetUserPreferenceToggle(C["swSTLPreview"], False)
    sw.SetUserPreferenceToggle(C["swSTLShowInfoOnSave"], False)
    sw.SetUserPreferenceIntegerValue(C["swExportStlUnits"], C["swMETER"])

    options = C["swOpenDocOptions_Silent"] | C["swOpenDocOptions_ReadOnly"]
    asm = sw.OpenDoc6(ASM, C["swDocASSEMBLY"], options, "", byref_int(), byref_int())
    title = call(asm, "GetTitle")
    log(f"Opened {title}")

    boxes = {}
    for comp in asm.GetComponents(False) or ():
        name = call(comp, "Name2")
        try:
            box = comp.GetBox(False, True)  # tight, without reference geometry; assembly coordinates, metres
            boxes[name] = list(box) if box else None
        except Exception as e:
            boxes[name] = None
            log(f"  GetBox {name}: {e}")
    with open(os.path.join(OUT, "boxes.json"), "w") as f:
        json.dump(boxes, f, indent=1)
    log(f"Boxes for {sum(1 for b in boxes.values() if b)} of {len(boxes)} components")

    # One STL per component, in assembly coordinates, coarse enough for a browser.
    sw.SetUserPreferenceToggle(C["swSTLComponentsIntoOneFile"], False)
    sw.SetUserPreferenceIntegerValue(C["swSTLQuality"], C["swSTLQuality_Coarse"])
    comp_dir = os.path.join(OUT, "components")
    os.makedirs(comp_dir, exist_ok=True)
    ok, err, warn = save_as(sw, asm, os.path.join(comp_dir, "cart.STL"))
    files = sorted(glob.glob(os.path.join(comp_dir, "*.STL")))
    log(f"Per-component export: ok={ok} errors={err} warnings={warn}; {len(files)} files, "
        f"{sum(os.path.getsize(p) for p in files) / 1e6:.1f} MB")

    # The arm's base part in its own frame, to line SolidWorks up with Umbratek's URDF.
    sw.SetUserPreferenceToggle(C["swSTLComponentsIntoOneFile"], True)
    sw.SetUserPreferenceIntegerValue(C["swSTLQuality"], C["swSTLQuality_Fine"])
    base_path = [os.path.join(os.path.dirname(os.path.dirname(ASM)), "UTRA_6-1000_3D_Model_V0.2", n + ".SLDPRT")
                 for n in ("1000-BASE", "1000-Link1")]
    for path in base_path:
        doc = sw.OpenDoc6(path, C["swDocPART"], options, "", byref_int(), byref_int())
        target = os.path.join(OUT, os.path.splitext(os.path.basename(path))[0] + "-part.STL")
        ok, err, warn = save_as(sw, doc, target)
        log(f"{os.path.basename(target)}: ok={ok} errors={err} warnings={warn} "
            f"{os.path.getsize(target) if os.path.exists(target) else 0} bytes")
    sw.CloseAllDocuments(True)
    log("Done; all documents closed without saving")


if __name__ == "__main__":
    pythoncom.CoInitialize()
    main()
