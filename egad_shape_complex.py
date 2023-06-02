import numpy as np
import trimesh
import sys
import os


def morphological_complexity(mesh):
    """
    Measure of morphological complexity from
    Environmental Influence on the Evolution of Morphological Complexity in Machines
    Joshua E. Auerbach, and Josh C. Bongard
    https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3879106/
    Given a mesh:
    1. Computes the angular deficit of the triangles at each vertex
    2. Compuates a histogram over the angular deficit
    3. Returns the entropy of the normalised histogram
    """
    # Compute internal angles per vertex
    flat_vertices = mesh.faces.flatten()
    flat_angles = mesh.face_angles.flatten()
    unique_vertices = np.unique(flat_vertices)
    vertex_angles = np.array([2*np.pi - flat_angles[flat_vertices == v_id].sum() for v_id in unique_vertices])
    # Create Normalised Histogram
    hist = np.histogram(vertex_angles, bins=512, range=(-np.pi*2, np.pi*2))[0].astype(np.float)
    hist /= hist.sum()
    # Compute entropy
    H = -1 * (hist * np.log2(hist+1e-6)).sum()
    H = max(0, H)
    return H


def main(args):
    mesh_path = args

    complexities = [None] * len(os.listdir(mesh_path))
    for mesh_name in os.listdir(mesh_path):
        if mesh_name.endswith('.ply'):
            path_to_mesh = os.path.join(mesh_path, mesh_name)
            theMesh = trimesh.load(path_to_mesh)
            #point_c = trimesh.load(path_to_mesh)
            #theMesh = trimesh.exchange.ply.load_ply(point_c, encoding='ascii')
            shape_complex = morphological_complexity(theMesh)
            complexities[int(mesh_name[4:-4])-1] = shape_complex
    return complexities


if __name__ == '__main__':
    comp1 = main(sys.argv[1])
    comp2 = main(sys.argv[2])

    print(os.path.split(sys.argv[1])[-1], os.path.split(sys.argv[2])[-1])
    for i in range(len(comp1)):
        print('obj ', i+1, ': ', comp1[i], comp2[i])
