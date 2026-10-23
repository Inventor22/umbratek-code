"""Export the UTRA cart assembly from SolidWorks (read-only) for the jog panel's 3D view and collision checks.

Attaches to a running SolidWorks, opens the assembly read-only and silently, records every component's file
and placement, saves each unique part as a binary STL in the part's own frame (metres), saves the whole
assembly as one STL in assembly coordinates as a cross-check, then closes the documents without saving
anything back.

Late-bound COM: pywin32 returns a zero-argument method's value on attribute access, so call() handles both.
"""
import json
import os
import sys
import time

import pythoncom
import win32com.client
from win32com.client import VARIANT

ASM = r"C:\Users\Dustin\Documents\Github\umbratek-code\..\umbratek-cad\Utra 1000\UTRA 1000 Aluminum Extrusion Assembly.SLDASM"
ASM = os.path.normpath(ASM)
SWCONST = r"C:\Program Files\Dassault Systemes\SOLIDWORKS 3DEXPERIENCE R2026x\SOLIDWORKS\swconst.tlb"
OUT = sys.argv[1]


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def call(obj, name, *args):
    member = getattr(obj, name)
    if isinstance(member, win32com.client.dynamic.CDispatch):  # a COM object (a property), not a method
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


def save_as(doc, path):
    errors, warnings = byref_int(), byref_int()
    ext = call(doc, "Extension")
    ok = ext.SaveAs(path, 0, 1, VARIANT(pythoncom.VT_DISPATCH, None), errors, warnings)
    if not ok or not os.path.exists(path):
        raise RuntimeError(f"SaveAs failed: errors={errors.value} warnings={warnings.value}")


def main():
    os.makedirs(os.path.join(OUT, "parts"), exist_ok=True)
    C = constants(SWCONST)
    sw = win32com.client.GetActiveObject("SldWorks.Application")
    log(f"Attached to SolidWorks {call(sw, 'RevisionNumber')}, {call(sw, 'GetDocumentCount')} documents open")

    # STL options: binary, metres, fine, part/assembly coordinates kept (not moved into positive space),
    # and no preview or info dialogs.
    sw.SetUserPreferenceToggle(C["swSTLBinaryFormat"], True)
    sw.SetUserPreferenceToggle(C["swSTLDontTranslateToPositive"], True)
    sw.SetUserPreferenceToggle(C["swSTLComponentsIntoOneFile"], True)
    sw.SetUserPreferenceToggle(C["swSTLPreview"], False)
    sw.SetUserPreferenceToggle(C["swSTLShowInfoOnSave"], False)
    sw.SetUserPreferenceIntegerValue(C["swExportStlUnits"], C["swMETER"])
    sw.SetUserPreferenceIntegerValue(C["swSTLQuality"], C["swSTLQuality_Fine"])

    options = C["swOpenDocOptions_Silent"] | C["swOpenDocOptions_ReadOnly"]
    errors, warnings = byref_int(), byref_int()
    asm = sw.OpenDoc6(ASM, C["swDocASSEMBLY"], options, "", errors, warnings)
    if asm is None:
        raise SystemExit(f"Couldn't open {ASM}: errors={errors.value} warnings={warnings.value}")
    title = call(asm, "GetTitle")
    log(f"Opened {title} (errors={errors.value} warnings={warnings.value})")
    try:
        asm.ResolveAllLightWeightComponents(False)
    except Exception as e:
        log(f"ResolveAllLightWeightComponents: {e}")

    components = []
    for comp in asm.GetComponents(False) or ():
        path = call(comp, "GetPathName")
        xf = call(comp, "Transform2")
        components.append({
            "name": call(comp, "Name2"),
            "path": path,
            "kind": "assembly" if path.lower().endswith(".sldasm") else "part",
            "suppressed": bool(call(comp, "IsSuppressed")),
            "hidden": bool(comp.IsHidden(True)),
            "transform": list(call(xf, "ArrayData")) if xf is not None else None,
        })
    log(f"{len(components)} components")
    with open(os.path.join(OUT, "components.json"), "w") as f:
        json.dump({"assembly": ASM, "components": components}, f, indent=1)

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
            part_title = call(doc, "GetTitle")
            if part_title != title:
                sw.CloseDoc(part_title)
    for c in components:
        c["stl"] = files.get(c["path"])

    whole = os.path.join(OUT, "assembly.STL")
    try:
        sw.ActivateDoc3(title, False, 0, byref_int())
        save_as(asm, whole)
        log(f"assembly.STL  {os.path.getsize(whole)} bytes")
    except Exception as e:
        log(f"assembly STL: {e}")

    with open(os.path.join(OUT, "components.json"), "w") as f:
        json.dump({"assembly": ASM, "solidworks": call(sw, "RevisionNumber"), "components": components}, f, indent=1)
    sw.CloseDoc(title)
    log(f"Done: {len(files)} part STLs for {len(components)} components; documents closed without saving")


if __name__ == "__main__":
    pythoncom.CoInitialize()
    main()
