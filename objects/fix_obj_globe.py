import math

def add_uvs_to_obj(input_path, output_path):
    vertices = []
    lines = []

    with open(input_path, "r") as f:
        lines = f.readlines()

    # Extract vertex coordinates
    for line in lines:
        if line.startswith("v "):
            parts = list(map(float, line.strip().split()[1:4]))
            vertices.append(parts)

    # Calculate spherical UV projection (u, v)
    uvs = []
    for x, y, z in vertices:
        radius = math.sqrt(x**2 + y**2 + z**2)
        if radius == 0:
            uvs.append((0.0, 0.0))
            continue
        u = 0.5 + (math.atan2(z, x) / (2 * math.pi))
        v = 0.5 + (math.asin(max(-1.0, min(1.0, y / radius))) / math.pi)
        uvs.append((u, v))

    # Write modified OBJ output
    with open(output_path, "w") as f:
        vt_written = False
        for line in lines:
            if line.startswith("v "):
                f.write(line)
            elif line.startswith("f "):
                if not vt_written:
                    for u, v in uvs:
                        f.write(f"vt {u:.6f} {v:.6f}\n")
                    vt_written = True
                
                parts = line.strip().split()[1:]
                new_face = []
                for p in parts:
                    indices = p.split("/")
                    v_idx = indices[0]
                    vn_idx = indices[2] if len(indices) > 2 else ""
                    new_face.append(f"{v_idx}/{v_idx}/{vn_idx}".rstrip("/"))
                f.write("f " + " ".join(new_face) + "\n")
            elif line.startswith("vt "):
                continue
            else:
                f.write(line)

add_uvs_to_obj("globe.obj", "globe_fixed.obj")