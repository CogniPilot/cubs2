#!/usr/bin/env python3
"""
Improve GLB mesh visibility by adding emission/ambient lighting to materials.
Usage: blender --background --python improve_lighting.py
"""

import bpy
import sys
from pathlib import Path

# Configuration
MESH_FILE = Path(__file__).parent / "concrete_floor.glb"
OUTPUT_FILE = Path(__file__).parent / "concrete_floor.glb"
EMISSION_STRENGTH = 0.8  # Much higher for visibility from all angles
MIX_FACTOR = 0.5  # 50/50 mix for consistent brightness from all angles


def improve_material_lighting():
    """Add emission shader to all materials for better visibility."""

    # Clear existing scene
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    # Import GLB
    print(f"Loading: {MESH_FILE}")
    bpy.ops.import_scene.gltf(filepath=str(MESH_FILE))

    # Process all materials
    materials_modified = 0
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            print(f"Skipping {mat.name} - not using nodes")
            continue

        nodes = mat.node_tree.nodes
        links = mat.node_tree.links

        # Find the material output node
        output_node = None
        for node in nodes:
            if node.type == "OUTPUT_MATERIAL":
                output_node = node
                break

        if not output_node:
            print(f"Warning: No output node found for {mat.name}")
            continue

        # Find what's currently connected to the output
        existing_shader = None
        for link in links:
            if link.to_node == output_node and link.to_socket.name == "Surface":
                existing_shader = link.from_node
                existing_socket = link.from_socket
                # Remove the existing connection
                links.remove(link)
                break

        if not existing_shader:
            print(f"Warning: No existing shader found for {mat.name}")
            continue

        # Create emission shader
        emission = nodes.new(type="ShaderNodeEmission")
        emission.inputs["Strength"].default_value = EMISSION_STRENGTH
        emission.inputs["Color"].default_value = (1.0, 1.0, 1.0, 1.0)  # White
        emission.location = (
            existing_shader.location[0],
            existing_shader.location[1] - 200,
        )

        # Create mix shader to blend emission with existing shader
        mix_shader = nodes.new(type="ShaderNodeMixShader")
        mix_shader.inputs["Fac"].default_value = MIX_FACTOR
        mix_shader.location = (
            existing_shader.location[0] + 200,
            existing_shader.location[1],
        )

        # Connect nodes: existing shader and emission to mix shader
        links.new(existing_socket, mix_shader.inputs[1])
        links.new(emission.outputs["Emission"], mix_shader.inputs[2])

        # Connect mix shader to output
        links.new(mix_shader.outputs["Shader"], output_node.inputs["Surface"])

        print(f"Modified material: {mat.name}")
        materials_modified += 1

    # Also increase base color brightness for Principled BSDF nodes
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            continue
        for node in mat.node_tree.nodes:
            if node.type == "BSDF_PRINCIPLED":
                # Get current base color
                current_color = node.inputs["Base Color"].default_value
                # Brighten significantly for side visibility (multiply by 2.0)
                new_color = [min(c * 2.0, 1.0) for c in current_color[:3]]
                node.inputs["Base Color"].default_value = (*new_color, current_color[3])
                # Reduce metallic for better diffuse lighting
                node.inputs["Metallic"].default_value = 0.0
                # Increase roughness slightly
                if "Roughness" in node.inputs:
                    node.inputs["Roughness"].default_value = min(
                        node.inputs["Roughness"].default_value * 1.2, 1.0
                    )
                print(f"Adjusted Principled BSDF in {mat.name}")

    # Export as GLB
    print(f"\nExporting to: {OUTPUT_FILE}")
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT_FILE),
        export_format="GLB",
        export_materials="EXPORT",
        export_colors=True,
        export_normals=True,
        export_apply=False,
    )

    print(f"\nSuccess! Modified {materials_modified} materials")
    print(f"Emission strength: {EMISSION_STRENGTH}")
    print(f"Mix factor: {MIX_FACTOR}")


if __name__ == "__main__":
    improve_material_lighting()
