import numpy as np
import trimesh
import sys
import os
import open3d as o3d
import matplotlib.pyplot as plt

#############################################
# shamelessly stolen from: https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def morphological_complexity(mesh_path):
    # trimesh.geometry.vector_angle(pairs)
    # Find the angles between pairs of unit vectors.
    # Parameters: pairs ((n, 2, 3) float) – Unit vector pairs
    # Returns: angles – Angles between vectors in radians
    # Return type: (n,) float
    bins = 256
    radius = 1.5

    #pcd = o3d.io.read_point_cloud(mesh_path)
    #o3d.visualization.draw_geometries([pcd])
    #downpcd = o3d.geometry.voxel_down_sample(pcd, voxel_size=0.01)
    #o3d.visualization.draw_geometries([downpcd])
    #pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=1000))
    #pcd_normals = np.array(pcd.normals)
    #print('norm: ', pcd_normals.shape)

    #pcd = o3d.io.read_point_cloud(mesh_path)
    #o3d.visualization.draw_geometries([pcd])
    #downpcd = pcd.voxel_down_sample(voxel_size=3.0)
    #downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5.0, max_nn=8))
    #o3d.visualization.draw_geometries([downpcd])
    #radii = [6.0, 9.0, 12.0, 20.0]
    #triangles = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(downpcd, o3d.utility.DoubleVector(radii))
    #o3d.visualization.draw_geometries([triangles])
    triangles = o3d.io.read_triangle_mesh(mesh_path)
    triangles = triangles.simplify_vertex_clustering(voxel_size=radius)
    if not triangles.has_adjacency_list():
        triangles = triangles.compute_adjacency_list()
    if not triangles.has_triangle_normals():
        triangles = triangles.compute_triangle_normals()
    triangles.normalize_normals()

    #theAdjacencies = triangles.cluster_connected_triangles()
    theAdjacencies = np.array(triangles.adjacency_list)
    theNormals = np.asarray(triangles.triangle_normals)
    #theVertices = np.asarray(triangles.vertices)
    theTriangles = np.asarray(triangles.triangles)
    #print(np.array(theAdjacencies[0]).shape)
    #print('Normals: ', theNormals.shape)
    #print(theVertices.shape)
    #print('Triangles: ', theTriangles.shape)

    flat_angles = []
    already_considered = []
    for idx in range(theAdjacencies.shape[0]):
        currentAdj = theAdjacencies[idx]
        for neighb in currentAdj:
            if neighb in already_considered:
                continue
            that_angle = angle_between(theNormals[idx, :], theNormals[neighb, :])
            # center and shift!
            #that_angle = (that_angle - np.pi/2) * 4
            #that_angle = that_angle / np.pi
            flat_angles.append(that_angle)
        already_considered.append(idx)
        #print('adjs: ', currentAdj)
        #that_angle = angle_between()
    vertex_angles = np.array(flat_angles)

    #print('angles: ', vertex_angles.shape)
    #sys.exit()
    #mesh = trimesh.load(mesh_path)
    # Compute internal angles per vertex
    #flat_vertices = mesh.faces.flatten()
    #flat_angles = mesh.face_angles.flatten()
    #unique_vertices = np.unique(flat_vertices)
    #vertex_angles = np.array([2*np.pi - flat_angles[flat_vertices == v_id].sum() for v_id in unique_vertices])
    # Create Normalised Histogram
    #hist = np.histogram(vertex_angles, bins=32, range=(0, np.pi))[0].astype(np.float)
    #hist = np.histogram(vertex_angles, bins=16, range=(-2*np.pi, 2*np.pi))[0].astype(np.float64)

    hist = np.histogram(vertex_angles, bins=bins, range=(0.0, np.pi))[0].astype(np.float64)
    #hist /= hist.sum()
    hist /= np.max(hist)

    #plt.hist(range(bins), bins, weights=hist)
    #plt.show()
    #H_norm = np.repeat(np.array([0.5]), axis=0, repeats=bins)
    #H_norm = -1 * (H_norm * np.log2(H_norm+1e-6)).sum()

    #print('hist: ', hist)

    # compute integral
    H = np.sum(hist)
    # Compute entropy
    #H = -1 * (hist * np.log2(hist+1e-6)).sum()
    H = max(0, H)
    return H


def main(args):
    mesh_path = args

    complexities = [None] * (len(os.listdir(mesh_path))-1)
    if len(os.listdir(mesh_path)) == 16: # linemod
        complexities = [None] * 13
    for mesh_name in os.listdir(mesh_path):
        if mesh_name.endswith('.ply'):
            if len(os.listdir(mesh_path)) == 16 and int(mesh_name[4:-4]) in [3, 7]:
                continue
            path_to_mesh = os.path.join(mesh_path, mesh_name)
            #theMesh = trimesh.load(path_to_mesh)

            # radius downsample
            #print(len(theMesh.faces))
            #theMesh.show()
            #_, valid_indexes = trimesh.sample.sample_surface_even(theMesh, len(theMesh.faces), radius=0.005)
            #print(len(valid_indexes))
            #theMesh = theMesh.submesh([valid_indexes])[0]
            #theMesh.show()

            print('Processing object ', int(mesh_name[4:-4]))
            shape_complex = morphological_complexity(path_to_mesh)
            if len(os.listdir(mesh_path)) == 16 and int(mesh_name[4:-4]) > 7:
                complexities[int(mesh_name[4:-4]) - 3] = shape_complex
            elif len(os.listdir(mesh_path)) == 16 and int(mesh_name[4:-4]) > 3:
                complexities[int(mesh_name[4:-4]) - 2] = shape_complex
            else:
                complexities[int(mesh_name[4:-4])-1] = shape_complex
    return complexities


if __name__ == '__main__':
    comp1 = main(sys.argv[1])
    comp2 = main(sys.argv[2])

    #print(os.path.split(sys.argv[1])[-1], os.path.split(sys.argv[2])[-1])
    for i in range(len(comp1)):
        print('obj ', i+1, ': ', comp1[i])#, comp2[i])

    tless_o = np.array([23.2, 32.4, 38.5, 21.0, 85.8, 84.7, 82.0, 80.0, 86.6, 81.8, 64.6, 83.5, 57.9, 69.2, 72.6, 61.7, 93.2, 91.8, 71.7, 60.8, 62.9, 60.1, 69.0, 79.7, 81.1, 78.8, 91.7, 61.5, 87.8, 59.7])
    tless_r = np.array([25.6, 31.3, 36.7, 24.1, 93.7, 87.8, 91.2, 98.0, 95.1, 93.0, 66.3, 82.7, 55.7, 72.6, 74.7, 63.8, 95.2, 93.2, 69.6, 51.3, 65.1, 58.0, 72.2, 77.1, 83.2, 86.9, 86.5, 68.8, 89.8, 84.7])
    tless_d = tless_r / tless_o

    lm_o = np.array([59.5, 84.0, 56.0, 81.5, 59.5, 77.0, 28.5, 69.5, 65.0, 21.0, 86.0, 82.5, 36.5])
    lm_r = np.array([40.5, 77.5, 36.0, 77.5, 59.5, 67.5, 32.0, 49.5, 57.5, 23.5, 64.0, 77.5, 47.5])
    lm_d = lm_r / lm_o

    deltas = np.concatenate([lm_d, tless_d])
    compx = np.concatenate([comp1, comp2])

    sort_comp = np.argsort(compx)
    sorted_plex = compx[sort_comp]
    sorted_delta = deltas[sort_comp]

    plt.plot(sorted_plex, sorted_delta)
    plt.show()