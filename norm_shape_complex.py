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
    triangles = triangles.simplify_vertex_clustering(voxel_size=3.0)
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
            that_angle = that_angle / np.pi
            flat_angles.append(that_angle)
        already_considered.append(idx)
        #print('adjs: ', currentAdj)
        #that_angle = angle_between()
    vertex_angles = np.array(flat_angles)

    #print('angles: ', vertex_angles.shape)
    #print('min/max: ', np.min(vertex_angles), np.max(vertex_angles))
    #sys.exit()
    #mesh = trimesh.load(mesh_path)
    # Compute internal angles per vertex
    #print(mesh.faces.shape)
    #flat_vertices = mesh.faces.flatten()
    #print(flat_vertices.shape)
    #flat_angles = mesh.face_angles.flatten()
    #print('angle: ', flat_angles.shape)
    #unique_vertices = np.unique(flat_vertices)
    #vertex_angles = np.array([2*np.pi - flat_angles[flat_vertices == v_id].sum() for v_id in unique_vertices])
    # Create Normalised Histogram
    #hist = np.histogram(vertex_angles, bins=32, range=(0, np.pi))[0].astype(np.float)
    #hist = np.histogram(vertex_angles, bins=512, range=(-2*np.pi, 2*np.pi))[0].astype(np.float64)
    hist = np.histogram(vertex_angles, bins=512, range=(0, 1.0))[0].astype(np.float)
    hist /= hist.sum()

    plt.hist(range(512), 512, weights=hist)
    plt.show()

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
            #theMesh = trimesh.load(path_to_mesh)

            # radius downsample
            #print(len(theMesh.faces))
            #theMesh.show()
            #_, valid_indexes = trimesh.sample.sample_surface_even(theMesh, len(theMesh.faces), radius=0.005)
            #print(len(valid_indexes))
            #theMesh = theMesh.submesh([valid_indexes])[0]
            #theMesh.show()

            print('Processing object ', int(mesh_name[4:-4])-1)
            shape_complex = morphological_complexity(path_to_mesh)
            complexities[int(mesh_name[4:-4])-1] = shape_complex
    return complexities


if __name__ == '__main__':
    comp1 = main(sys.argv[1])
    comp2 = main(sys.argv[2])

    print(os.path.split(sys.argv[1])[-1], os.path.split(sys.argv[2])[-1])
    for i in range(len(comp1)):
        print('obj ', i+1, ': ', comp1[i], comp2[i])