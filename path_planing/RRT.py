import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple

class RRT:
    """Rapidly-exploring Random Tree (RRT) algorithm"""

    def __init__(self, origin: np.ndarray, width: float, height: float, circular_obstacles: List[Tuple[np.ndarray, float]], rectangular_obstacles: List[Tuple[np.ndarray, float, float]]) -> None:
        """
        Initialize the RRT algorithm.

        All the measures are in centimeters.

        @param origin: The starting configuration of the robot.
        @param width: The width of the configuration space.
        @param height: The height of the configuration space.
        """
        self.origin = origin
        self.width = width
        self.height = height
        self.circular_obstacles = circular_obstacles
        self.rectangular_obstacles = rectangular_obstacles

        # if there are rectangular obstacles in the configuration space, a matrix is created to represent the obstacles
        if self.rectangular_obstacles is not None:
            self.rectangular_obstacles_matrix = np.zeros((self.width, self.height))
            for obstacle in self.rectangular_obstacles:
                x, y, w, h = obstacle
                self.rectangular_obstacles_matrix[x:x+w, y:y+h] = 1


    def plan(self, trials: int=1000, step_size: float=0.2) -> (np.ndarray, np.ndarray):
        """
        Explore a workspace using the RRT algorithm.
        
        This function builds an RRT using `trials` samples from the free space.
        
        @param obstacles: A list of circular or rectaangular obstacles.
        @param trials: The number of configurations to sample from the free space.
        @param step_size: The step_size to pass to `extend`.
        
        @return: A tuple (`vertices`, `parents`), where `vertices` is an (n, 2) `np.ndarray` where each row is a configuration vertex
                and `parents` is an array identifying the parent, i.e. `parents[i]` is the parent of the vertex in
                the `i`th row of `vertices.
        """
        num_verts = 1
        
        vertices = np.zeros((trials + 1, len(origin)))
        vertices[0, :] = origin
        
        parents = np.zeros(trials + 1, dtype=int)
        parents[0] = -1
        
        for trial in range(trials):
            pass
    
        return vertices[:num_verts, :], parents[:num_verts]

    def backtrack(self, index: int, parents: np.ndarray) -> List[int]:
        """
        Find the sequence of nodes from the origin of the graph to an index.
        
        This function returns a List of vertex indices going from the origin vertex to the vertex `index`.
        
        @param index: The vertex to find the path through the tree to.
        @param parents: The array of vertex parents as specified in the `rrt` function.
        
        @return: The list of vertex indicies such that specifies a path through the graph to `index`.
        """
        
        pass

    def nearest_vertex(self, conf: np.ndarray, vertices: np.ndarray) -> int:
        """
        Finds the nearest vertex to conf in the set of vertices.
        
        This function searches through the set of vertices and finds the one that is closest to 
        conf using the L2 norm (Euclidean distance).
        
        @param conf: The configuration we are trying to find the closest vertex to.
        @param vertices: The set of vertices represented as an np.array with shape (n, 2). Each row represents
                        a vertex.
        @return: The index (i.e. row of `vertices`) of the vertex that is closest to `conf`.
        """
        pass

    def extend(self, target: np.ndarray, step_size: float=0.2) -> np.ndarray:
        """
        Extends the RRT at most a fixed distance toward the target configuration.
        
        Given a configuration in the RRT graph `origin`, this function returns a new configuration that takes a
        step of at most `step_size` towards the `target` configuration. That is, if the L2 distance between `origin`
        and `target` is less than `step_size`, return `target`. Otherwise, return the configuration on the line
        segment between `origin` and `target` that is `step_size` distance away from `origin`.
        
        @param origin: A vertex in the RRT graph to be extended.
        @param target: The vertex that is being extended towards.
        @param step_size: The maximum allowed distance the returned vertex can be from `origin`.
        
        @return: A new configuration that is as close to `target` as possible without being more than
                `step_size` away from `origin`.

        """
        pass

    def random_conf(self) -> np.ndarray:
        """
        Sample a random configuration from the configuration space.
        
        This function draws a uniformly random configuration from the configuration space rectangle. The configuration 
        does not necessarily have to reside in the free space.
        
        @return: A random configuration uniformily distributed across the configuration space.
        """
        pass

    def conf_free(self, q: np.ndarray) -> bool:
        """
        Check if a configuration is in the free space.
        
        This function checks if the configuration q lies outside of all the obstacles in the connfiguration space.
        
        @param q: An np.ndarray of shape (2,) representing a robot configuration.
        @return: True if the configuration is in the free space. 
                Otherwise return False.
        """
        # check if the configuration is in the rectangular obstacles
        if self.rectangular_obstacles is not None:
            x, y = q
            if self.rectangular_obstacles_matrix[x, y] == 1:
                return False
        
        # check if obstacle is in the circular obstacles
        if self.circular_obstacles is not None:
            for obstacle in self.circular_obstacles:
                center, radius = obstacle
                if np.linalg.norm(q - center) < radius:
                    return False
                
        return True

    def edge_free(self, edge: Tuple[np.ndarray, np.ndarray]) -> bool:
        """
        Check if a graph edge is in the free space.
        
        This function checks if a graph edge, i.e. a line segment specified as two end points, lies entirely outside of
        every obstacle in the configuration space.
        
        @param edge: A tuple containing the two segment endpoints.
        @return: True if the edge is in the free space. 
                Otherwise return False.
        """
        x1, y1 = edge[0]
        x2, y2 = edge[1]
        
        # check if edge intersects with rectangular obstacles
        if self.rectangular_obstacles is not None:
            if x1 == x2:
                for obstacle in self.rectangular_obstacles:
                    x, y, w, h = obstacle
                    if x == x1 and y <= y1 <= y+h:
                        return False
            elif y1 == y2:
                for obstacle in self.rectangular_obstacles:
                    x, y, w, h = obstacle
                    if y == y1 and x <= x1 <= x+w:
                        return False
            else:
                for obstacle in self.rectangular_obstacles:
                    x, y, w, h = obstacle
                    if x <= x1 <= x+w and y <= y1 <= y+h:
                        return False
                    if x <= x2 <= x+w and y <= y2 <= y+h:
                        return False
        
        # check if edge intersects with circular obstacles
        if self.circular_obstacles is not None:
            _distance_line_origin = np.abs((y1-self.origin[1]) * (x2-x1) - (x1-self.origin[0]) *(y2-y1)) / np.sqrt((y2-y1)**2 + (x2-x1)**2)
            pass

        pass