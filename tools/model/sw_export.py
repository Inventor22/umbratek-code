"""Export the UTRA cart assembly from SolidWorks (read-only) for the jog panel's 3D view and collision checks.

Attaches to a running SolidWorks (waiting for it to finish starting), opens the assembly read-only and
silently, records every component's file and placement, saves each unique part as a binary STL in the
part's own frame (metres), saves the whole assembly as one STL in assembly coordinates as a cross-check,
then closes the documents without saving anything back.
"""
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
WAIT_S = float(sys.argv[2]) if len(sys.argv) > 2 else 900


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


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


def attach():
    """The running SolidWorks, wrapped with its type library so methods and properties behave as documented."""
    deadline = time.monotonic() + WAIT_S
    while True:
        try:
            running = win32com.client.GetActiveObject("SldWorks.Application")
            break
        except pythoncom.com_error as e:  # not running or still starting (sign-in, splash)
            if time.monotonic() > deadline:
                raise SystemExit(f"SolidWorks didn't become available within {WAIT_S:.0f} s: {e}")
            time.sleep(5)
    return win32com.client.gencache.EnsureDispatch(running)


def save_as(doc, path):
    errors, warnings = byref_int(), byref_int()
    ok = doc.Extension.SaveAs(path, 0, 1, VARIANT(pythoncom.VT_DISPATCH, None), errors, warnings)
    if not ok or not os.path.exists(path):
        raise RuntimeError(f"SaveAs failed for {path}: errors={errors.value} warnings={warnings.value}")


def main():
    os.makedirs(os.path.join(OUT, "parts"), exist_ok=True)
    C = constants(SWCONST)
    log("STL-related constants: " + ", ".join(sorted(k for k in C if "STL" in k.upper() or "Stl" in k)))
    sw = attach()
    log(f"Attached to SolidWorks {sw.RevisionNumber()}")
    sw.Visible = True

    # STL options: binary, metres, fine, part/assembly coordinates kept (not moved into positive space).
    sw.SetUserPreferenceToggle(C["swSTLBinaryFormat"], True)
    sw.SetUserPreferenceToggle(C["swSTLDontTranslateToPositive"], True)
    sw.SetUserPreferenceToggle(C["swSTLComponentsIntoOneFile"], True)
    sw.SetUserPreferenceToggle(C["swSTLPreview"], False)  # no dialogs mid-export
    sw.SetUserPreferenceToggle(C["swSTLShowInfoOnSave"], False)
    sw.SetUserPreferenceIntegerValue(C["swExportStlUnits"], C["swMETER"])
    sw.SetUserPreferenceIntegerValue(C["swSTLQuality"], C["swSTLQuality_Fine"])

    errors, warnings = byref_int(), byref_int()
    options = C["swOpenDocOptions_Silent"] | C["swOpenDocOptions_ReadOnly"]
    asm = sw.OpenDoc6(ASM, C["swDocASSEMBLY"], options, "", errors, warnings)
    if asm is None:
        raise SystemExit(f"Couldn't open the assembly: errors={errors.value} warnings={warnings.value}")
    log(f"Opened assembly (errors={errors.value} warnings={warnings.value}); resolving lightweight parts")
    try:
        asm.ResolveAllLightWeightComponents(False)
    except Exception as e:
        log(f"ResolveAllLightWeightComponents: {e}")

    components = []
    for comp in asm.GetComponents(False) or ():
        path = comp.GetPathName()
        xf = comp.Transform2
        components.append({
            "name": comp.Name2,
            "path": path,
            "kind": "assembly" if path.lower().endswith(".sldasm") else "part",
            "suppressed": bool(comp.IsSuppressed()),
            "hidden": bool(comp.IsHidden(True)),
            "transform": list(xf.ArrayData) if xf is not None else None,
        })
    log(f"{len(components)} components")

    parts = sorted({c["path"] for c in components if c["kind"] == "part" and not c["suppressed"]})
    files = {}
    for path in parts:
        stem = os.path.splitext(os.path.basename(path))[0]
        target = os.path.join(OUT, "parts", stem + ".STL")
        errors, warnings = byref_int(), byref_int()
        doc = sw.OpenDoc6(path, C["swDocPART"], options, "", errors, warnings)
        if doc is None:
            log(f"  couldn't open {stem}: errors={errors.value}")
            continue
        try:
            save_as(doc, target)
            files[path] = os.path.basename(target)
            log(f"  {stem}.STL  {os.path.getsize(target)} bytes")
        except Exception as e:
            log(f"  {stem}: {e}")
        finally:
            if doc.GetTitle() != asm.GetTitle():
                sw.CloseDoc(doc.GetTitle())
    for c in components:
        c["stl"] = files.get(c["path"])

    whole = os.path.join(OUT, "assembly.STL")
    try:
        sw.ActivateDoc3(asm.GetTitle(), False, 0, byref_int())
        save_as(asm, whole)
        log(f"assembly.STL  {os.path.getsize(whole)} bytes")
    except Exception as e:
        log(f"assembly STL: {e}")

    with open(os.path.join(OUT, "components.json"), "w") as f:
        json.dump({"assembly": ASM, "solidworks": sw.RevisionNumber(), "components": components}, f, indent=1)
    sw.CloseDoc(asm.GetTitle())
    log(f"Done: {len(files)} parts, {len(components)} components; documents closed without saving")


if __name__ == "__main__":
    pythoncom.CoInitialize()
    main()
