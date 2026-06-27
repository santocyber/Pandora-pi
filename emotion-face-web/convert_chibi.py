import os

import bpy


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BLEND_PATH = os.path.join(BASE_DIR, "chibi", "source", "Free Chibi Girl AKi.blend")
TEXTURE_DIR = os.path.join(BASE_DIR, "chibi", "textures")
GLB_PATH = os.path.join(BASE_DIR, "static", "models", "chibi.glb")


def texture_index(texture_dir):
    files = {}
    for name in os.listdir(texture_dir):
        path = os.path.join(texture_dir, name)
        if os.path.isfile(path):
            files[name.lower()] = path
    return files


def relink_images(texture_dir):
    textures = texture_index(texture_dir)
    relinked = 0
    missing = []

    for img in bpy.data.images:
        if img.source not in {"FILE", "SEQUENCE"}:
            continue

        current = bpy.path.abspath(img.filepath) if img.filepath else ""
        filename = os.path.basename(current or img.name).lower()
        candidate = textures.get(filename)

        if not candidate:
            stem = os.path.splitext(filename)[0]
            for tex_name, tex_path in textures.items():
                if os.path.splitext(tex_name)[0] == stem:
                    candidate = tex_path
                    break

        if candidate:
            img.filepath = candidate
            try:
                img.reload()
            except RuntimeError as exc:
                print(f"WARN image reload failed: {img.name}: {exc}")
            relinked += 1
        elif current and not os.path.exists(current):
            missing.append(img.name)

    print(f"Images relinked from {texture_dir}: {relinked}")
    if missing:
        print("Missing images not found in texture folder:")
        for name in missing:
            print(f"  - {name}")


def make_local_data():
    for datablock in list(bpy.data.materials) + list(bpy.data.images) + list(bpy.data.textures):
        if getattr(datablock, "library", None):
            try:
                datablock.make_local()
            except RuntimeError as exc:
                print(f"WARN make_local failed: {datablock.name}: {exc}")


def configure_materials():
    clip_kw = ["FACE", "BROW", "LASH", "EYELINE", "MOUTH", "TOOTH"]

    for mat in bpy.data.materials:
        upper = mat.name.upper()
        mat.use_nodes = True

        if "HAIR" in upper:
            mat.blend_method = "CLIP"
            mat.alpha_threshold = 0.35
            mat.show_transparent_back = True
            mat.use_screen_refraction = False

        elif "EYEWHITE" in upper or "EYEIRIS" in upper:
            mat.blend_method = "OPAQUE"
            mat.use_screen_refraction = False

        elif "EYEHIGHLIGHT" in upper:
            mat.blend_method = "BLEND"
            mat.show_transparent_back = False
            mat.use_screen_refraction = False

        elif "SKIN" in upper:
            mat.blend_method = "OPAQUE"
            mat.use_screen_refraction = False

        elif any(k in upper for k in clip_kw):
            mat.blend_method = "CLIP"
            mat.alpha_threshold = 0.35
            mat.show_transparent_back = True
            mat.use_screen_refraction = False
        else:
            mat.blend_method = "OPAQUE"


def print_scene_report():
    objs = bpy.context.scene.objects
    meshes = [o for o in objs if o.type == "MESH"]
    armatures = [o for o in objs if o.type == "ARMATURE"]
    print(f"Objects: {len(objs)}, Meshes: {len(meshes)}, Armatures: {len(armatures)}")

    for m in meshes:
        sk = m.data.shape_keys
        if sk:
            names = [kb.name for kb in sk.key_blocks if kb.name != "Basis"]
            fcl = [n for n in names if n.startswith("Fcl_")]
            print(f"  {m.name}: {len(names)} shape keys, {len(fcl)} Fcl_")

    for arm in armatures:
        print(f"  {arm.name}: {len(arm.data.bones)} bones")


def export_glb(glb_path):
    os.makedirs(os.path.dirname(glb_path), exist_ok=True)

    kwargs = {
        "filepath": glb_path,
        "export_format": "GLB",
        "export_texcoords": True,
        "export_normals": True,
        "export_materials": "EXPORT",
        "export_morph": True,
        "export_morph_normal": True,
        "export_skins": True,
        "export_animations": True,
        "export_image_format": "AUTO",
        "export_extras": True,
    }

    optional = {
        "export_tangents": True,
        "export_morph_tangent": True,
        "export_yup": True,
    }

    supported = {prop.identifier for prop in bpy.ops.export_scene.gltf.get_rna_type().properties}
    for key, value in optional.items():
        if key in supported:
            kwargs[key] = value
        else:
            print(f"Exporter option not supported by this Blender: {key}")

    print(f"\nExporting GLB: {glb_path}")
    bpy.ops.export_scene.gltf(**kwargs)


def main():
    print(f"Opening: {BLEND_PATH}")
    bpy.ops.wm.open_mainfile(filepath=BLEND_PATH)

    make_local_data()
    relink_images(TEXTURE_DIR)
    configure_materials()
    print_scene_report()
    export_glb(GLB_PATH)
    print("Done")


if __name__ == "__main__":
    main()
