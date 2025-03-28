import numpy as np
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans

class BPS_Encoder:
    def __init__(self, environment_points, num_basis_points=256):
        self.environment_points = np.array(environment_points)
        self.num_basis_points = num_basis_points
        self.basis_points = None
        self.encoding = None

    def generate_basis_points(self):
        min_coords = self.environment_points.min(axis=0)
        max_coords = self.environment_points.max(axis=0)
        self.basis_points = np.random.rand(self.num_basis_points, 2) * (max_coords - min_coords) + min_coords

    def generate_basis_points_grid(self):
        min_coords = self.environment_points.min(axis=0)
        max_coords = self.environment_points.max(axis=0)
        grid_size = int(np.sqrt(self.num_basis_points))
        x_lin = np.linspace(min_coords[0], max_coords[0], grid_size)
        y_lin = np.linspace(min_coords[1], max_coords[1], grid_size)
        xv, yv = np.meshgrid(x_lin, y_lin)
        self.basis_points = np.vstack([xv.ravel(), yv.ravel()]).T

    def generate_basis_points_kmeans(self):
        kmeans = KMeans(n_clusters=self.num_basis_points, random_state=42)
        kmeans.fit(self.environment_points)
        self.basis_points = kmeans.cluster_centers_

    def compute_encoding(self, feature_type='distance', basis_point_method='random', **kwargs):
        if basis_point_method == 'grid':
            self.generate_basis_points_grid()
        elif basis_point_method == 'random':
            self.generate_basis_points()
        elif basis_point_method == 'kmeans':
            self.generate_basis_points_kmeans()
        else:
            raise ValueError("Unsupported basis point generation method.")

        tree = cKDTree(self.environment_points)
        distances, _ = tree.query(self.basis_points, k=1)
        if feature_type == 'distance':
            max_distance = kwargs.get('max_distance', distances.max())
            if max_distance == 0 or np.isnan(max_distance):
                self.encoding = np.zeros_like(distances)
            else:
                self.encoding = distances / max_distance
        elif feature_type == 'occupancy':
            threshold = kwargs.get('threshold', 0.1)
            self.encoding = (distances < threshold).astype(float)
        else:
            raise ValueError("Unsupported feature_type")
        return self.encoding

    def get_encoding(self):
        if self.encoding is None:
            raise ValueError("Encoding has not been computed yet.")
        return self.encoding

    # def visualize_encoding(self):
    #     if self.encoding is None:
    #         raise ValueError("Encoding has not been computed yet.")

    #     plt.scatter(self.basis_points[:, 0], self.basis_points[:, 1],
    #                 c=self.encoding, cmap='viridis')
    #     plt.colorbar(label='Encoding Value')
    #     plt.title("BPS Encoding Visualization")
    #     plt.show()
