import math
import os
import numpy as np
import matplotlib.pyplot as plt

from collections import deque

from geometry_msgs.msg import PoseStamped

from tb4_exploration.base_explorer import BaseExplorer
from tb4_slam_utils.map_utils import analyze_map


class FrontierExplorer(BaseExplorer):
    def __init__(self, node):
        super().__init__(node)

        self.recent_goals = []
        self.max_recent_goals = 5
        self.revisit_radius = 1.5

    def get_name(self):
        return 'frontier'

    def start(self):
        self.is_running = True
        self.node.get_logger().info('Starting Frontier Explorer...')

    def stop(self):
        self.is_running = False
        self.node.get_logger().info('Stopping Frontier Explorer...')

    def cluster_frontiers(self, frontier_grid_cells):
        frontier_set = set(frontier_grid_cells)
        visited = set()
        clusters = []

        directions = [
            (-1, -1), (0, -1), (1, -1),
            (-1,  0),          (1,  0),
            (-1,  1), (0,  1), (1,  1),
        ]

        for cell in frontier_grid_cells:
            if cell in visited:
                continue

            cluster = []
            queue = deque([cell])
            visited.add(cell)

            while queue:
                current = queue.popleft()
                cluster.append(current)

                cx, cy = current

                for dx, dy in directions:
                    neighbor = (cx + dx, cy + dy)

                    if neighbor in frontier_set and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)

            clusters.append(cluster)

        return clusters
    
    def select_largest_frontier_cluster_goal(
        self,
        frontier_grid_cells,
        origin_x,
        origin_y,
        resolution,
        robot_x,
        robot_y,
        min_cluster_size=20
    ):
        clusters = self.cluster_frontiers(frontier_grid_cells)

        valid_clusters = [
            cluster for cluster in clusters
            if len(cluster) >= min_cluster_size
        ]

        if not valid_clusters:
            return None, {
                'clusters': clusters,
                'valid_clusters': valid_clusters,
                'selected_cluster': None,
                'selected_cluster_size': 0,
                'selected_distance': None,
            }

        largest_cluster = max(valid_clusters, key=len)

        centroid_x = sum(cell[0] for cell in largest_cluster) / len(largest_cluster)
        centroid_y = sum(cell[1] for cell in largest_cluster) / len(largest_cluster)

        best_grid_cell = min(
            largest_cluster,
            key=lambda cell: math.hypot(
                cell[0] - centroid_x,
                cell[1] - centroid_y
            )
        )

        gx, gy = best_grid_cell

        goal_x = origin_x + (gx + 0.5) * resolution
        goal_y = origin_y + (gy + 0.5) * resolution

        selected_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)

        return (goal_x, goal_y), {
            'clusters': clusters,
            'valid_clusters': valid_clusters,
            'selected_cluster': largest_cluster,
            'selected_cluster_size': len(largest_cluster),
            'selected_grid_cell': best_grid_cell,
            'selected_distance': selected_distance,
        }
    
    def select_best_frontier_cluster_goal(
        self,
        frontier_grid_cells,
        map_msg,
        robot_x,
        robot_y,
        min_cluster_size=20,
        information_radius=1.5,
        unknown_weight=1.0,
        cluster_size_weight=0.2,
        distance_weight=10.0,
        revisit_penalty_value=500.0
    ):
        """
        Select the best frontier cluster using a utility score.

        score =
            unknown_weight * unknown_count
            + cluster_size_weight * cluster_size
            - distance_weight * robot_distance
            - revisit_penalty

        Returns:
            selected_cell:
                (goal_x, goal_y) in world coordinates, or None

            selection_info:
                Debug information about all clusters and selected result
        """

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        data = map_msg.data

        clusters = self.cluster_frontiers(frontier_grid_cells)

        valid_clusters = [
            cluster
            for cluster in clusters
            if len(cluster) >= min_cluster_size
        ]

        selection_info = {
            'clusters': clusters,
            'valid_clusters': valid_clusters,
            'candidate_results': [],
            'selected_cluster': None,
            'selected_cluster_size': 0,
            'selected_grid_cell': None,
            'selected_distance': None,
            'selected_unknown_count': 0,
            'selected_revisit_penalty': 0.0,
            'selected_score': None,
        }

        if not valid_clusters:
            return None, selection_info

        radius_cells = max(
            1,
            int(information_radius / resolution)
        )

        best_score = float('-inf')
        best_world_cell = None
        best_grid_cell = None
        best_cluster = None
        best_distance = None
        best_unknown_count = 0
        best_revisit_penalty = 0.0

        for cluster in valid_clusters:
            # Compute cluster centroid in grid coordinates
            centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
            centroid_y = sum(cell[1] for cell in cluster) / len(cluster)

            # Pick an actual frontier cell closest to centroid
            representative_cell = min(
                cluster,
                key=lambda cell: math.hypot(
                    cell[0] - centroid_x,
                    cell[1] - centroid_y
                )
            )

            grid_x, grid_y = representative_cell

            goal_x = origin_x + (grid_x + 0.5) * resolution
            goal_y = origin_y + (grid_y + 0.5) * resolution

            robot_distance = math.hypot(
                goal_x - robot_x,
                goal_y - robot_y
            )

            # Count unknown cells around the representative point
            unknown_count = 0

            min_x = max(0, grid_x - radius_cells)
            max_x = min(width - 1, grid_x + radius_cells)
            min_y = max(0, grid_y - radius_cells)
            max_y = min(height - 1, grid_y + radius_cells)

            radius_squared = radius_cells * radius_cells

            for y in range(min_y, max_y + 1):
                for x in range(min_x, max_x + 1):
                    dx = x - grid_x
                    dy = y - grid_y

                    # Use a circular information-gain region
                    if dx * dx + dy * dy > radius_squared:
                        continue

                    index = y * width + x

                    if data[index] == -1:
                        unknown_count += 1

            # Penalize clusters close to recently selected goals
            revisit_penalty = 0.0
            nearest_recent_goal_distance = None

            for recent_goal_x, recent_goal_y in self.recent_goals:
                recent_distance = math.hypot(
                    goal_x - recent_goal_x,
                    goal_y - recent_goal_y
                )

                if (
                    nearest_recent_goal_distance is None
                    or recent_distance < nearest_recent_goal_distance
                ):
                    nearest_recent_goal_distance = recent_distance

                if recent_distance < self.revisit_radius:
                    revisit_penalty = revisit_penalty_value
                    break

            score = (
                unknown_weight * unknown_count
                + cluster_size_weight * len(cluster)
                - distance_weight * robot_distance
                - revisit_penalty
            )

            candidate_result = {
                'cluster': cluster,
                'cluster_size': len(cluster),
                'representative_grid_cell': representative_cell,
                'goal_world_cell': (goal_x, goal_y),
                'distance': robot_distance,
                'unknown_count': unknown_count,
                'revisit_penalty': revisit_penalty,
                'nearest_recent_goal_distance': nearest_recent_goal_distance,
                'score': score,
            }

            selection_info['candidate_results'].append(candidate_result)

            self.node.get_logger().info(
                f"Frontier cluster candidate: "
                f"size={len(cluster)}, "
                f"unknown={unknown_count}, "
                f"distance={robot_distance:.2f}, "
                f"revisit_penalty={revisit_penalty:.1f}, "
                f"score={score:.2f}, "
                f"goal=({goal_x:.2f}, {goal_y:.2f})"
            )

            if score > best_score:
                best_score = score
                best_world_cell = (goal_x, goal_y)
                best_grid_cell = representative_cell
                best_cluster = cluster
                best_distance = robot_distance
                best_unknown_count = unknown_count
                best_revisit_penalty = revisit_penalty

        if best_world_cell is None:
            return None, selection_info

        selection_info.update({
            'selected_cluster': best_cluster,
            'selected_cluster_size': len(best_cluster),
            'selected_grid_cell': best_grid_cell,
            'selected_distance': best_distance,
            'selected_unknown_count': best_unknown_count,
            'selected_revisit_penalty': best_revisit_penalty,
            'selected_score': best_score,
        })

        # Remember selected goal to discourage immediate revisits
        self.recent_goals.append(best_world_cell)

        if len(self.recent_goals) > self.max_recent_goals:
            self.recent_goals.pop(0)

        return best_world_cell, selection_info

    def compute_next_goal(self, map_msg, robot_tf):
        self.node.get_logger().info("DEBUG: compute_next_goal entered")

        if map_msg is None or robot_tf is None:
            return None

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        data = map_msg.data

        min_goal_distance = 0.8  # meters
        min_cluster_size = 20

        stats = analyze_map(map_msg)

        self.node.get_logger().info(
            f"Map stats: width={stats['width']}, "
            f"height={stats['height']}, "
            f"free={stats['free']}, "
            f"occupied={stats['occupied']}, "
            f"unknown={stats['unknown']}"
        )

        robot_x = robot_tf.transform.translation.x
        robot_y = robot_tf.transform.translation.y

        frontier_world_cells = []
        frontier_grid_cells = []
        too_close_cells = []
        valid_cells = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                index = y * width + x

                # Frontier must be a free cell
                if data[index] != 0:
                    continue

                # Check unknown neighbor using 8-neighbor
                has_unknown_neighbor = False

                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue

                        nx = x + dx
                        ny = y + dy
                        n_index = ny * width + nx

                        if data[n_index] == -1:
                            has_unknown_neighbor = True
                            break

                    if has_unknown_neighbor:
                        break

                if not has_unknown_neighbor:
                    continue

                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution

                dist = math.hypot(world_x - robot_x, world_y - robot_y)

                frontier_grid_cells.append((x, y))
                frontier_world_cells.append((world_x, world_y))

                if dist < min_goal_distance:
                    too_close_cells.append((world_x, world_y))
                else:
                    valid_cells.append((world_x, world_y))

        # selected_cell, cluster_info = self.select_largest_frontier_cluster_goal(
        #     frontier_grid_cells,
        #     origin_x,
        #     origin_y,
        #     resolution,
        #     robot_x,
        #     robot_y,
        #     min_cluster_size=min_cluster_size
        # )

        # self.node.get_logger().info(
        #     f"Frontier candidates={len(frontier_grid_cells)}, "
        #     f"clusters={len(cluster_info['clusters'])}, "
        #     f"valid_clusters={len(cluster_info['valid_clusters'])}, "
        #     f"too_close={len(too_close_cells)}, "
        #     f"valid={len(valid_cells)}, "
        #     f"min_goal_distance={min_goal_distance:.2f}m"
        # )

        selected_cell, cluster_info = self.select_best_frontier_cluster_goal(
            frontier_grid_cells=frontier_grid_cells,
            map_msg=map_msg,
            robot_x=robot_x,
            robot_y=robot_y,
            min_cluster_size=20,
            information_radius=1.5,
            unknown_weight=1.0,
            cluster_size_weight=0.2,
            distance_weight=10.0,
            revisit_penalty_value=500.0
        )

        self.node.get_logger().info(
            f"Frontier candidates={len(frontier_grid_cells)}, "
            f"clusters={len(cluster_info['clusters'])}, "
            f"valid_clusters={len(cluster_info['valid_clusters'])}"
        )

        self.visualize_debug(
            map_msg,
            robot_tf,
            frontier_world_cells,
            too_close_cells,
            valid_cells,
            selected_cell,
            min_goal_distance
        )

        if selected_cell is None:
            self.node.get_logger().warn('No valid frontier cluster found.')
            return None

        goal = PoseStamped()
        goal.header.frame_id = map_msg.header.frame_id if map_msg.header.frame_id else 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()

        goal.pose.position.x = selected_cell[0]
        goal.pose.position.y = selected_cell[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        # self.node.get_logger().info(
        #     f"Largest frontier cluster selected: "
        #     f"cluster_size={cluster_info['selected_cluster_size']}, "
        #     f"x={selected_cell[0]:.2f}, "
        #     f"y={selected_cell[1]:.2f}, "
        #     f"dist={cluster_info['selected_distance']:.2f}"
        # )

        self.node.get_logger().info(
            f"Best frontier cluster selected: "
            f"cluster_size={cluster_info['selected_cluster_size']}, "
            f"unknown={cluster_info['selected_unknown_count']}, "
            f"score={cluster_info['selected_score']:.2f}, "
            f"x={selected_cell[0]:.2f}, "
            f"y={selected_cell[1]:.2f}, "
            f"dist={cluster_info['selected_distance']:.2f}"
        )

        return goal
    

    def visualize_debug(
        self,
        map_msg,
        robot_tf,
        frontier_cells,
        too_close_cells,
        valid_cells,
        selected_cell,
        min_goal_distance
    ):
        self.node.get_logger().info("DEBUG: calling visualize_debug()")

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        data = np.array(map_msg.data).reshape((height, width))

        robot_x = robot_tf.transform.translation.x
        robot_y = robot_tf.transform.translation.y

        extent = [
            origin_x,
            origin_x + width * resolution,
            origin_y,
            origin_y + height * resolution
        ]

        plt.clf()

        # plt.imshow(
        #     data,
        #     cmap='gray_r',
        #     origin='lower',
        #     extent=extent,
        #     vmin=-1,
        #     vmax=100
        # )

        if frontier_cells:
            xs = [p[0] for p in frontier_cells]
            ys = [p[1] for p in frontier_cells]
            plt.scatter(xs, ys, s=30, marker='o', label='frontier')

        if too_close_cells:
            xs = [p[0] for p in too_close_cells]
            ys = [p[1] for p in too_close_cells]
            plt.scatter(xs, ys, s=60, marker='x', label='too close')

        if valid_cells:
            xs = [p[0] for p in valid_cells]
            ys = [p[1] for p in valid_cells]
            plt.scatter(xs, ys, s=40, marker='o', label='valid')

        if selected_cell is not None:
            plt.scatter(
                [selected_cell[0]],
                [selected_cell[1]],
                s=150,
                marker='*',
                label='selected goal'
            )

        plt.scatter(
            [robot_x],
            [robot_y],
            s=150,
            marker='P',
            label='robot'
        )

        circle = plt.Circle(
            (robot_x, robot_y),
            min_goal_distance,
            fill=False,
            linestyle='--',
            label=f'min distance {min_goal_distance:.2f}m'
        )

        ax = plt.gca()
        ax.add_patch(circle)

        plt.title(
            f'Frontier Debug | '
            f'frontiers={len(frontier_cells)}, '
            f'too_close={len(too_close_cells)}, '
            f'valid={len(valid_cells)}'
        )

        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()

        debug_dir = os.path.expanduser('~/frontier_debug')
        os.makedirs(debug_dir, exist_ok=True)

        save_path = os.path.join(debug_dir, 'frontier_debug_latest.png')
        plt.savefig(save_path)

        # plt.show(block=False)
        plt.pause(0.01)

        self.node.get_logger().info(f'Debug map saved to: {save_path}')