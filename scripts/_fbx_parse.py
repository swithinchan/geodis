import ufbx, json, sys, os

fbx_path = sys.argv[1]
fname = os.path.basename(fbx_path)

try:
    scene = ufbx.load_file(fbx_path)
    for node in scene.nodes:
        if node.mesh is None:
            continue
        mesh = node.mesh
        m = node.node_to_world

        verts = []
        for i in range(mesh.num_vertices):
            v = mesh.vertices[i]
            wx = m.c0.x*v.x + m.c1.x*v.y + m.c2.x*v.z + m.c3.x
            wy = m.c0.y*v.x + m.c1.y*v.y + m.c2.y*v.z + m.c3.y
            wz = m.c0.z*v.x + m.c1.z*v.y + m.c2.z*v.z + m.c3.z
            verts.extend([wx / 100.0, wy / 100.0, wz / 100.0])

        # Sanity check: HK80 coordinates for HK should be > 100000 meters
        if verts[0] < 50000 or verts[1] < 50000:
            print(json.dumps({"ok": False, "error": f"coords out of HK80 range: ({verts[0]:.0f},{verts[1]:.0f})"}))
            sys.exit(0)

        # Skip degenerate geometry (buildings with near-zero footprint)
        xmin = float('inf'); xmax = float('-inf')
        ymin = float('inf'); ymax = float('-inf')
        for i in range(0, len(verts), 3):
            if verts[i] < xmin: xmin = verts[i]
            if verts[i] > xmax: xmax = verts[i]
            if verts[i+1] < ymin: ymin = verts[i+1]
            if verts[i+1] > ymax: ymax = verts[i+1]
        if (xmax - xmin) < 0.5 and (ymax - ymin) < 0.5:
            print(json.dumps({"ok": False, "error": f"degenerate footprint: {xmax-xmin:.2f}x{ymax-ymin:.2f}m"}))
            sys.exit(0)

        idx = [int(mesh.vertex_indices[i]) for i in range(mesh.num_indices)]

        zmin = float('inf'); zmax = float('-inf')
        for i in range(0, len(verts), 3):
            z = verts[i+2]
            if z < zmin: zmin = z
            if z > zmax: zmax = z

        print(json.dumps({
            "ok": True, "v": verts, "i": idx,
            "base_pd": zmin, "roof_pd": zmax
        }))
        sys.exit(0)

    print(json.dumps({"ok": False, "error": "no mesh node"}))
except Exception as e:
    print(json.dumps({"ok": False, "error": str(e)}))
