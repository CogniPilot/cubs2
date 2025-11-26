#!/usr/bin/env python3
# Copyright 2025 CogniPilot Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Improve GLB mesh visibility by adding emission/ambient lighting to materials.

Usage: blender --background --python improve_lighting.py
"""
from pathlib import Path

import bpy

# Configuration
MESH_FILE = Path(__file__).parent / 'plane.glb'
OUTPUT_FILE = Path(__file__).parent / 'plane.glb'
EMISSION_STRENGTH = 0.4  # Adjust this value (0.3-0.6 recommended)
MIX_FACTOR = 0.25  # How much emission vs original shader (0.2-0.3 recommended)


def improve_material_lighting():
    """Add emission shader to all materials for better visibility."""
    # Clear existing scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

    # Import GLB
    print(f'Loading: {MESH_FILE}')
    bpy.ops.import_scene.gltf(filepath=str(MESH_FILE))

    # Process all materials
    materials_modified = 0
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            print(f'Skipping {mat.name} - not using nodes')
            continue

        nodes = mat.node_tree.nodes
        links = mat.node_tree.links

        # Find the material output node
        output_node = None
        for node in nodes:
            if node.type == 'OUTPUT_MATERIAL':
                output_node = node
                break

        if not output_node:
            print(f'Warning: No output node found for {mat.name}')
            continue

        # Find what's currently connected to the output
        existing_shader = None
        for link in links:
            if link.to_node == output_node and link.to_socket.name == 'Surface':
                existing_shader = link.from_node
                existing_socket = link.from_socket
                # Remove the existing connection
                links.remove(link)
                break

        if not existing_shader:
            print(f'Warning: No existing shader found for {mat.name}')
            continue

        # Create emission shader
        emission = nodes.new(type='ShaderNodeEmission')
        emission.inputs['Strength'].default_value = EMISSION_STRENGTH
        emission.inputs['Color'].default_value = (1.0, 1.0, 1.0, 1.0)  # White
        emission.location = (
            existing_shader.location[0],
            existing_shader.location[1] - 200,
        )

        # Create mix shader to blend emission with existing shader
        mix_shader = nodes.new(type='ShaderNodeMixShader')
        mix_shader.inputs['Fac'].default_value = MIX_FACTOR
        mix_shader.location = (
            existing_shader.location[0] + 200,
            existing_shader.location[1],
        )

        # Connect nodes: existing shader and emission to mix shader
        links.new(existing_socket, mix_shader.inputs[1])
        links.new(emission.outputs['Emission'], mix_shader.inputs[2])

        # Connect mix shader to output
        links.new(mix_shader.outputs['Shader'], output_node.inputs['Surface'])

        print(f'Modified material: {mat.name}')
        materials_modified += 1

    # Also increase base color brightness for Principled BSDF nodes
    for mat in bpy.data.materials:
        if not mat.use_nodes:
            continue
        for node in mat.node_tree.nodes:
            if node.type == 'BSDF_PRINCIPLED':
                # Get current base color
                current_color = node.inputs['Base Color'].default_value
                # Brighten it (multiply by 1.3, cap at 1.0)
                new_color = [min(c * 1.3, 1.0) for c in current_color[:3]]
                node.inputs['Base Color'].default_value = (
                    *new_color, current_color[3])
                # Reduce metallic for better diffuse lighting
                node.inputs['Metallic'].default_value = 0.0
                # Increase roughness slightly
                if 'Roughness' in node.inputs:
                    node.inputs['Roughness'].default_value = min(
                        node.inputs['Roughness'].default_value * 1.2, 1.0
                    )
                print(f'Adjusted Principled BSDF in {mat.name}')

    # Export as GLB
    print(f'\nExporting to: {OUTPUT_FILE}')
    bpy.ops.export_scene.gltf(
        filepath=str(OUTPUT_FILE),
        export_format='GLB',
        export_materials='EXPORT',
        export_colors=True,
        export_normals=True,
        export_apply=False,
    )

    print(f'\nSuccess! Modified {materials_modified} materials')
    print(f'Emission strength: {EMISSION_STRENGTH}')
    print(f'Mix factor: {MIX_FACTOR}')


if __name__ == '__main__':
    improve_material_lighting()
