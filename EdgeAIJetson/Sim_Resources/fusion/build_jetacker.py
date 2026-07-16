# fusion/build_jetacker.py
"""Build the JetAcker sim model in Fusion 360 from params.json.

Run via the Fusion MCP execute tool. Rebuilds from scratch each run
(direct-modeling design; the 'parametric' source of truth is params.json).
All geometry is in car-assembly coordinates: base_link frame == Fusion origin
(rear axle center, axle height). Fusion API lengths are in cm: meters * 100.
Set EXPORT=True (Task 6) to write STLs to ../meshes/.
"""
import json
import os
import adsk.core
import adsk.fusion

SIM = ("/Users/nathanjones/Documents/Arduino/Mining Research Mother File/"
       "Mining-Research/EdgeAIJetson/Sim_Resources")
M = 100.0        # meters -> Fusion-internal cm
EXPORT = False   # flipped to True in Task 6

AXES = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}


def _pt(x, y, z):
    return adsk.core.Point3D.create(x * M, y * M, z * M)


def build(root, params):
    brep = adsk.fusion.TemporaryBRepManager.get()
    occs = {}

    def new_link(name):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        occs[name] = occ
        return occ.component

    def add_box(comp, pos, size, name):
        obb = adsk.core.OrientedBoundingBox3D.create(
            _pt(*pos),
            adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0),
            size[0] * M, size[1] * M, size[2] * M)
        comp.bRepBodies.add(brep.createBox(obb)).name = name

    def add_cyl(comp, pos, axis, radius, length, name):
        ax = AXES[axis]
        p1 = _pt(*(pos[i] - ax[i] * length / 2 for i in range(3)))
        p2 = _pt(*(pos[i] + ax[i] * length / 2 for i in range(3)))
        body = brep.createCylinderOrCone(p1, radius * M, p2, radius * M)
        comp.bRepBodies.add(body).name = name

    def add_part(comp, part):
        if part["type"] == "box":
            add_box(comp, part["pos"], part["size"], part["name"])
        else:
            add_cyl(comp, part["pos"], part["axis"], part["radius"],
                    part["length"], part["name"])

    wr, ww = params["wheel_radius"], params["wheel_width"]
    wb, tr = params["wheelbase"], params["track"]

    # base_link: chassis + all rigidly-mounted parts
    base = new_link("base_link")
    for part in params["base_parts"]:
        add_part(base, part)

    # wheels + front steering knuckles
    for side, sy in (("left", 1), ("right", -1)):
        kpos = (wb, sy * tr / 2, 0)
        knuckle = new_link("front_%s_steering_link" % side)
        add_cyl(knuckle, (kpos[0], kpos[1] - sy * 0.01, kpos[2]), "z", 0.010, 0.050, "knuckle")
        wheel = new_link("front_%s_wheel_link" % side)
        add_cyl(wheel, kpos, "y", wr, ww, "tire")
        add_cyl(wheel, kpos, "y", wr * 0.55, ww * 1.1, "hub")
        wheel = new_link("rear_%s_wheel_link" % side)
        add_cyl(wheel, (0, sy * tr / 2, 0), "y", wr, ww, "tire")
        add_cyl(wheel, (0, sy * tr / 2, 0), "y", wr * 0.55, ww * 1.1, "hub")

    # fixed sensor links
    for name, s in params["sensor_links"].items():
        comp = new_link(name)
        add_part(comp, {**s["shape"], "pos": s["pos"], "name": name})

    return occs


def run(_context):
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    design.fusionUnitsManager.distanceDisplayUnits = \
        adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    root = design.rootComponent

    with open(os.path.join(SIM, "params.json")) as f:
        params = json.load(f)

    occs = build(root, params)

    # report: link list + overall envelope in mm (union of body bboxes; cm -> mm)
    # NOTE: occurrence.boundingBox reads empty for direct-modeling temp-BRep
    # bodies, so union the per-body boxes instead.
    mins = [1e30] * 3
    maxs = [-1e30] * 3
    for occ in occs.values():
        for body in occ.bRepBodies:
            bb = body.boundingBox
            for i, (lo, hi) in enumerate([(bb.minPoint.x, bb.maxPoint.x),
                                          (bb.minPoint.y, bb.maxPoint.y),
                                          (bb.minPoint.z, bb.maxPoint.z)]):
                mins[i] = min(mins[i], lo)
                maxs[i] = max(maxs[i], hi)
    size_mm = [(maxs[i] - mins[i]) * 10 for i in range(3)]
    print("links: %s" % sorted(occs.keys()))
    print("envelope LxWxH mm: %.1f x %.1f x %.1f" % tuple(size_mm))
    print("ground clearance note: model z=0 is axle height; ground at z=%.1f mm"
          % (-params["wheel_radius"] * 1000))
    app.activeViewport.fit()

    if EXPORT:
        export_stls(design, occs)


def export_stls(design, occs):
    mesh_dir = os.path.join(SIM, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    mgr = design.exportManager
    for name, occ in occs.items():
        path = os.path.join(mesh_dir, name + ".stl")
        opts = mgr.createSTLExportOptions(occ, path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        mgr.execute(opts)
        print("exported %s" % path)
