# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================

import trimesh

import os
from pathlib import Path
from tabulate import tabulate
from termcolor import colored

##################################################################################################
############################################# Helpers ############################################
##################################################################################################

def _load_mesh(mesh_file: Path) -> trimesh.Trimesh:
    '''
    Load a mesh from a file. (Mainly for syntax highlighting later on)

    Args:
        mesh_file (Path): Path to the STL file.

    Returns:
        Trimesh: Loaded mesh object.
    '''
    if not mesh_file.is_file():
        raise FileNotFoundError(f"{mesh_file} is not a valid file.")
    
    return trimesh.load_mesh(mesh_file, force='mesh')

##################################################################################################
########################################### List faces ###########################################
##################################################################################################

def list_nb_faces(directory: Path) -> None:
    '''
    List the number of faces in all STL files in a directory.

    Args:
        directory (Path): Directory containing STL files.
    '''
    directory = Path(directory)
    if not directory.is_dir():
        raise NotADirectoryError(f"{directory} is not a valid directory.")

    entries = []
    for file in directory.iterdir():
        if file.suffix.lower() == ".stl" and file.is_file():
            try:
                mesh = _load_mesh(file)
                face_count = mesh.faces.shape[0]
                entries.append((file.name, face_count))
            except Exception as e:
                entries.append((file.name, f"Error: {e}"))

    if not entries:
        print("No STL files found.")
        return

    # Pad list to make it even
    if len(entries) % 2 != 0:
        entries.append(("", ""))

    # Pair rows side-by-side
    side_by_side = []
    for i in range(0, len(entries), 2):
        left = entries[i]
        right = entries[i + 1]
        side_by_side.append([
            left[0], left[1],
            right[0], right[1]
        ])

    headers = ["Filename", "Face Count", "Filename", "Face Count"]
    print(tabulate(side_by_side, headers=headers, tablefmt="grid"))

##################################################################################################
########################################## Shift meshes ##########################################
##################################################################################################

def shift_mesh(stl_file: Path, amount: float = 0.0, axis: str = 'x', verbose: bool = True) -> None:
    '''
    Shift a mesh along a specified axis by a given amount.

    Args:
        stl_file (Path): Path to the STL file to shift.
        amount (float):  Amount to shift in the specified axis.
        axis (str):      Axis to shift along ('x', 'y', or 'z'). Default is 'x'.
        verbose (bool):  If True, print a success message. Default is True.
    '''
    stl_file = Path(stl_file)
    if not stl_file.is_file():
        raise FileNotFoundError(f"{stl_file} is not a valid file.")
    
    if axis not in ['x', 'y', 'z']:
        raise ValueError(f"Invalid axis '{axis}'. Options are 'x', 'y', or 'z'.")
    
    axis2index = {'x': 0, 'y': 1, 'z': 2}
    idx = axis2index[axis]

    if verbose and amount == 0.0:
        print(colored(f"Warning: No shift applied to '{stl_file.name}' and no new file exported.", "yellow"))
        return
    elif amount < 0.0:
        shift_text = f"m{abs(int(amount))}"
    elif amount > 0.0:
        shift_text = str(int(amount))
    output_dir = stl_file.parent.parent / f"{stl_file.parent.stem}_shifted_{shift_text}_{axis}"
    os.makedirs(output_dir, exist_ok=True)

    mesh = _load_mesh(stl_file)
    mesh.vertices[:, idx] += amount

    mesh.export(output_dir / stl_file.name)
    if verbose:
        print(colored(f"Shifted '{stl_file.name}' by {amount} along {axis}-axis and saved it to: {output_dir / stl_file.name}", "green"))

def shift_meshes(mesh_dir: Path, amount: float = 0.0, axis: str = 'x') -> None:
    '''
    Shift all STL files in a directory along a specified axis by a given amount.

    Args:
        mesh_dir (Path): Directory containing the STL files to shift.
        amount (float):  Amount to shift in the specified axis.
        axis (str):      Axis to shift along ('x', 'y', or 'z'). Default is 'x'.
    '''
    mesh_dir = Path(mesh_dir)
    if not mesh_dir.is_dir():
        raise NotADirectoryError(f"{mesh_dir} is not a valid directory.")
    
    if axis not in ['x', 'y', 'z']:
        raise ValueError(f"Invalid axis '{axis}'. Options are 'x', 'y', or 'z'.")

    for stl_file in mesh_dir.rglob("*.stl"):
        shift_mesh(stl_file, amount, axis, verbose=False)

    if amount == 0.0:
        print(colored(f"Warning: No shift applied to any STL files in '{mesh_dir}' and no new files exported.", "yellow"))
        return
    elif amount < 0.0:
        shift_text = f"m{abs(int(amount))}"
    elif amount > 0.0:
        shift_text = str(int(amount))
    output_dir = mesh_dir.parent / f"{mesh_dir.stem}_shifted_{shift_text}_{axis}"
    print(colored(f"Shifted all STL files in '{mesh_dir}' by {amount} along the {axis}-axis into '{output_dir}'", "green"))

##################################################################################################
######################################### Mirror meshes ##########################################
##################################################################################################

def mirror_mesh(stl_file: Path, plane: str = 'yz', verbose: bool = True) -> None:
    '''
    Mirror a mesh around a given plane.

    Args:
        stl_file (Path): Path to the STL file to mirror.
        plane (str):     Plane to mirror across. Options are 'yz' (default), 'xz' and 'xy'.
        verbose (bool):  If True, print a success message. Default is True.
    '''
    stl_file = Path(stl_file)
    if not stl_file.is_file():
        raise FileNotFoundError(f"{stl_file} is not a valid file.")
    
    output_dir = stl_file.parent.parent / f"{stl_file.parent.stem}_mirrored_{plane}"
    os.makedirs(output_dir, exist_ok=True)
    
    if plane not in ['yz', 'xz', 'xy']:
        raise ValueError(f"Invalid plane '{plane}'. Options are 'yz' (default), 'xz', or 'xy'.")
    plane2index = {'yz': 0, 'xz': 1, 'xy': 2}

    mesh = _load_mesh(stl_file)
    mesh.vertices[:, plane2index[plane]] *= -1
    mesh.invert()

    mesh.export(output_dir / stl_file.name)
    if verbose:
        print(colored(f"Mirrored '{stl_file.name}' and saved it to: {output_dir / stl_file.name}", "green"))

def mirror_meshes(mesh_dir: Path, plane: str = 'yz') -> None:
    '''
    Mirror all STL files in a directory around a given plane.

    Args:
        mesh_dir (Path): Directory containing the STL files to mirror.
        plane (str):     Plane to mirror across. Options are 'yz' (default), 'xz', and 'xy'.
    '''
    mesh_dir = Path(mesh_dir)
    if not mesh_dir.is_dir():
        raise NotADirectoryError(f"{mesh_dir} is not a valid directory.")
    
    if plane not in ['yz', 'xz', 'xy']:
        raise ValueError(f"Invalid plane '{plane}'. Options are 'yz' (default), 'xz', or 'xy'.")

    for stl_file in mesh_dir.rglob("*.stl"):
        mirror_mesh(stl_file, plane, verbose=False)

    output_dir = mesh_dir.parent / f"{mesh_dir.stem}_mirrored_{plane}"
    print(colored(f"Mirrored all STL files in '{mesh_dir}' around the {plane} plane into '{output_dir}'", "green"))

##################################################################################################
######################################### Reduce meshes ##########################################
##################################################################################################

def reduce_mesh(stl_file: Path, target_faces: int, verbose: bool = True) -> None:
    '''
    Copy and reduce an STL file to a target numer of faces.

    Args:
        stl_file (Path):    Path to the original STL file.
        target_faces (int): Target number of faces for the mesh.
        verbose (bool):     If True, print a success message. Default is True.
    '''
    stl_file = Path(stl_file)
    if not stl_file.is_file():
        raise FileNotFoundError(f"{stl_file} is not a valid file.")

    output_dir = stl_file.parent.parent / f"{stl_file.parent.stem}_{target_faces}"
    os.makedirs(output_dir, exist_ok=True)

    curr_mesh = _load_mesh(stl_file)
    curr_mesh_count = len(curr_mesh.faces)
    prev_mesh_count = curr_mesh_count + 1
    while curr_mesh_count > target_faces and prev_mesh_count != curr_mesh_count:
        prev_mesh_count = curr_mesh_count

        curr_mesh = curr_mesh.simplify_quadric_decimation(face_count=target_faces)
        curr_mesh_count = len(curr_mesh.faces)

    if curr_mesh_count > target_faces:
        print(colored(f"Warning: Reduced '{stl_file.name}' to its minimum of {len(curr_mesh.faces)} faces (original target: {target_faces}).", "yellow"))

    curr_mesh.export(output_dir / stl_file.name)
    if verbose:
        print(colored(f"Reduced '{stl_file.name}' to {len(curr_mesh.faces)} faces and saved it to: {output_dir / stl_file.name}", "green"))

def reduce_meshes(mesh_dir: Path, target_faces: int) -> None:
    '''
    Copy and reduce all STL files in a directory to a target numer of faces.

    Args:
        mesh_dir (Path):    Directory containing the original STL files.
        target_faces (int): Maximum number of faces for all meshes.
    '''
    mesh_dir = Path(mesh_dir)
    if not mesh_dir.is_dir():
        raise NotADirectoryError(f"{mesh_dir} is not a valid directory.")
    
    for stl_file in mesh_dir.rglob("*.stl"):
        reduce_mesh(stl_file, target_faces, verbose=False)

    output_dir = mesh_dir.parent / f"{mesh_dir.stem}_{target_faces}"
    print(colored(f"Reduced all STL files in '{mesh_dir}' to a maximum of {target_faces} faces into '{output_dir}'", "green"))

##################################################################################################
######################################### Example Usages #########################################
##################################################################################################

if __name__ == "__main__":
    ## List number of faces
    list_nb_faces(Path("meshes") / "right" / "visual")

    ## Reduce meshes
    reduce_mesh(Path("meshes") / "right" / "visual" / "visual_tower_main.stl", target_faces=15000)
    reduce_meshes(Path("meshes") / "right" / "visual", target_faces=15000)

    ## Mirror meshes
    mirror_mesh(Path("meshes") / "right" / "visual" / "visual_tower_main.stl", plane='yz')
    mirror_meshes(Path("meshes") / "right" / "visual", plane='yz')

    ## Shift meshes
    shift_mesh(Path("meshes") / "right" / "visual" / "visual_tower_hull.stl", amount=-81, axis='x')
    shift_meshes(Path("meshes") / "right" / "visual", amount=-81, axis='x')
