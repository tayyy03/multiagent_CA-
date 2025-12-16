import pygame
import heapq
from typing import List, Tuple, Set, Optional, Dict
from dataclasses import dataclass, field
from collections import defaultdict
import random

pygame.init()

CELL_SIZE = 50
GRID_WIDTH = 15
GRID_HEIGHT = 12
WINDOW_WIDTH = CELL_SIZE * GRID_WIDTH
WINDOW_HEIGHT = CELL_SIZE * GRID_HEIGHT

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 100, 255)
YELLOW = (255, 255, 0)
DARK_GRAY = (100, 100, 100)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
CYAN = (0, 255, 255)
PINK = (255, 192, 203)

ROBOT_COLORS = [
    (0, 100, 255),
    (255, 0, 128),
    (0, 200, 100),
    (255, 128, 0),
    (128, 0, 255),
]

@dataclass(order=True)
class Node:
    f: float
    g: float = field(compare=False)
    h: float = field(compare=False)
    pos: Tuple[int, int] = field(compare=False)
    time: int = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)

@dataclass
class Task:
    task_id: int
    pickup_pos: Tuple[int, int]
    goal_pos: Tuple[int, int]
    mode: str
    cargo_type: str
    assigned_robot: Optional[int] = None

@dataclass
class Bid:
    robot_id: int
    task_id: int
    cost: float

class ReservationTable:
    def __init__(self):
        self.reservations: Dict[int, Set[Tuple[int, int]]] = defaultdict(set)
        self.robot_reservations: Dict[int, List[Tuple[Tuple[int, int], int]]] = defaultdict(list)
        
    def add_reservation(self, pos: Tuple[int, int], time: int, robot_id: Optional[int] = None):
        self.reservations[time].add(pos)
        if robot_id is not None:
            self.robot_reservations[robot_id].append((pos, time))
    
    def remove_robot_reservations(self, robot_id: int):
        if robot_id in self.robot_reservations:
            for pos, time in self.robot_reservations[robot_id]:
                if time in self.reservations and pos in self.reservations[time]:
                    self.reservations[time].discard(pos)
            self.robot_reservations[robot_id].clear()
    
    def is_reserved(self, pos: Tuple[int, int], time: int, exclude_robot: Optional[int] = None) -> bool:
        if pos not in self.reservations.get(time, set()):
            return False
        
        if exclude_robot is None:
            return True
        
        for other_robot_id, reservations in self.robot_reservations.items():
            if other_robot_id != exclude_robot:
                if (pos, time) in reservations:
                    return True
        return False
    
    def check_vertex_conflict(self, pos: Tuple[int, int], time: int, exclude_robot: Optional[int] = None) -> bool:
        return self.is_reserved(pos, time, exclude_robot)
    
    def check_edge_conflict(self, from_pos: Tuple[int, int], to_pos: Tuple[int, int], 
                           time: int, exclude_robot: Optional[int] = None) -> bool:
        """Cek swap conflict: A->B dan B->A pada waktu sama"""
        for other_robot_id, reservations in self.robot_reservations.items():
            if exclude_robot is not None and other_robot_id == exclude_robot:
                continue
            
            # Cari apakah robot lain bergerak dari to_pos ke from_pos di waktu yang sama
            if (to_pos, time + 1) in reservations and (from_pos, time) in reservations:
                return True
        
        return False
    
    def clear(self):
        self.reservations.clear()
        self.robot_reservations.clear()

class Robot:
    def __init__(self, robot_id: int, start_pos: Tuple[int, int]):
        self.robot_id = robot_id
        self.pos = start_pos
        self.grid_pos = start_pos  # Posisi diskrit untuk pathfinding
        self.robot_x = float(start_pos[0])
        self.robot_y = float(start_pos[1])
        self.path: List[Tuple[int, int]] = []
        self.current_step = 0
        self.speed = 0.05  # Lebih lambat untuk smooth movement
        self.color = ROBOT_COLORS[robot_id % len(ROBOT_COLORS)]
        
        self.current_task: Optional[Task] = None
        self.rack_pos = None
        self.current_phase = "idle"
        self.has_cargo = False
        self.reached_goal = False
        self.moving = False
        self.wait_counter = 0
        self.loading_time = 60
        self.unloading_time = 60
        
        # Untuk sinkronisasi waktu dengan reservation
        self.path_start_time = 0
        self.current_time_step = 0
        
    def smooth_move(self) -> bool:
        if self.current_step >= len(self.path):
            return True

        target_x, target_y = self.path[self.current_step]

        # Gerak smooth tapi TIDAK boleh lintas grid sebelum waktunya
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = (dx**2 + dy**2)**0.5

        move_distance = min(self.speed, distance)
        self.robot_x += (dx / distance) * move_distance if distance > 0 else 0
        self.robot_y += (dy / distance) * move_distance if distance > 0 else 0

        # GRID & TIME hanya berubah di sini
        if distance < 0.01:
            self.robot_x = float(target_x)
            self.robot_y = float(target_y)
            self.grid_pos = (target_x, target_y)
            self.current_step += 1
            self.current_time_step += 1

        return False

    
    def wait_in_place(self):
        self.current_time_step += 1
        self.grid_pos = self.grid_pos  # eksplisit hold

    
    def reset_for_new_task(self):
        self.reached_goal = False
        self.path = []
        self.current_step = 0
        self.has_cargo = False
        self.wait_counter = 0
        self.moving = False
        self.current_time_step = 0
        self.path_start_time = 0

class CooperativeAStarSystem:
    def __init__(self, num_robots: int = 5):
        self.grid_size = (GRID_WIDTH, GRID_HEIGHT)
        self.obstacles: Set[Tuple[int, int]] = set()
        
        self.pickup_points = [(2, 0), (4, 0), (6, 0), (8, 0), (10, 0)]
        self.delivery_points = [(2, 11), (4, 11), (6, 11), (8, 11), (10, 11)]
        
        self.racks = {
            'A': {
                'positions': [(i, 3) for i in range(1, 6)],
                'type': 'A',
                'color': (255, 100, 100),
                'access_point': (3, 2)
            },
            'B': {
                'positions': [(i, 5) for i in range(1, 6)],
                'type': 'B',
                'color': (100, 100, 255),
                'access_point': (3, 4)
            },
            'C': {
                'positions': [(i, 7) for i in range(1, 6)],
                'type': 'C',
                'color': (100, 255, 100),
                'access_point': (3, 6)
            },
            'D': {
                'positions': [(i, 9) for i in range(1, 6)],
                'type': 'D',
                'color': (255, 255, 100),
                'access_point': (3, 8)
            },
            'E': {
                'positions': [(i, 3) for i in range(8, 13)],
                'type': 'E',
                'color': (255, 150, 255),
                'access_point': (10, 2)
            },
            'F': {
                'positions': [(i, 5) for i in range(8, 13)],
                'type': 'F',
                'color': (150, 255, 255),
                'access_point': (10, 4)
            },
            'G': {
                'positions': [(i, 7) for i in range(8, 13)],
                'type': 'G',
                'color': (255, 200, 150),
                'access_point': (10, 6)
            },
            'H': {
                'positions': [(i, 9) for i in range(8, 13)],
                'type': 'H',
                'color': (200, 150, 255),
                'access_point': (10, 8)
            }
        }
        
        self.create_default_obstacles()
        
        self.robots: List[Robot] = []
        for i in range(num_robots):
            start_pos = self.get_random_free_position()
            robot = Robot(i, start_pos)
            self.robots.append(robot)
        
        self.all_tasks: Dict[int, Task] = {}
        self.available_task_ids: Set[int] = set()
        self.completed_task_ids: Set[int] = set()
        self.generate_random_tasks(num_tasks=8)
        
        self.reservation_table = ReservationTable()
        
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("CNP Cooperative A* - Fixed Collision System")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 20)
        self.tiny_font = pygame.font.Font(None, 16)
        
        self.running = True
        self.all_completed = False
        
        self.conduct_cnp_bidding()
    
    def create_default_obstacles(self):
        for rack_label, rack_info in self.racks.items():
            for pos in rack_info['positions']:
                self.obstacles.add(pos)
    
    def get_random_free_position(self) -> Tuple[int, int]:
        occupied = set()
        occupied.update(self.obstacles)
        occupied.update(self.pickup_points)
        occupied.update(self.delivery_points)
        for rack_info in self.racks.values():
            occupied.add(rack_info['access_point'])
        
        for robot in self.robots:
            occupied.add(robot.grid_pos)
        
        while True:
            x = random.randint(0, GRID_WIDTH - 1)
            y = random.randint(0, GRID_HEIGHT - 1)
            if (x, y) not in occupied:
                return (x, y)
    
    def generate_random_tasks(self, num_tasks: int):
        cargo_types = list(self.racks.keys())
        modes = ["store", "retrieve"]
        
        for i in range(num_tasks):
            mode = random.choice(modes)
            cargo_type = random.choice(cargo_types)
            pickup = random.choice(self.pickup_points)
            delivery = random.choice(self.delivery_points)
            
            task = Task(
                task_id=i,
                pickup_pos=pickup,
                goal_pos=delivery,
                mode=mode,
                cargo_type=cargo_type
            )
            self.all_tasks[i] = task
            self.available_task_ids.add(i)
        
        print(f"\n=== Generated {num_tasks} Random Tasks ===")
        for task_id, task in self.all_tasks.items():
            print(f"Task {task_id}: {task.mode.upper()} {task.cargo_type} | "
                  f"Pickup: {task.pickup_pos} -> Delivery: {task.goal_pos}")
    
    def conduct_cnp_bidding(self):
        print("\n=== Starting CNP Bidding Process ===")
        
        all_bids: List[Bid] = []
        idle_robots = [r for r in self.robots if r.current_task is None]
        
        for robot in idle_robots:
            for task_id in self.available_task_ids:
                task = self.all_tasks[task_id]
                cost = self.calculate_bid_cost(robot, task)
                bid = Bid(robot_id=robot.robot_id, task_id=task_id, cost=cost)
                all_bids.append(bid)
        
        all_bids.sort(key=lambda b: b.cost)
        
        assigned_robots = set()
        assigned_tasks = set()
        
        for bid in all_bids:
            if bid.robot_id not in assigned_robots and bid.task_id not in assigned_tasks:
                robot = self.robots[bid.robot_id]
                task = self.all_tasks[bid.task_id]
                
                robot.current_task = task
                task.assigned_robot = bid.robot_id
                
                assigned_robots.add(bid.robot_id)
                assigned_tasks.add(bid.task_id)
                
                print(f"Robot {robot.robot_id + 1} won Task {task.task_id} "
                      f"({task.mode} {task.cargo_type}) with cost: {bid.cost:.2f}")
        
        for task_id in assigned_tasks:
            self.available_task_ids.discard(task_id)
        
        print(f"\nRemaining unassigned tasks: {len(self.available_task_ids)}")
        
        for robot in self.robots:
            if robot.current_task is not None:
                self.plan_robot_path(robot)
    
    def calculate_bid_cost(self, robot: Robot, task: Task) -> float:
        rack_pos = self.get_rack_access_point(task.cargo_type)
        
        if task.mode == "store":
            path1 = self.cooperative_astar(robot.grid_pos, task.pickup_pos, 0, robot.robot_id)
            path2 = self.cooperative_astar(task.pickup_pos, rack_pos, len(path1), robot.robot_id)

            if not path1 or not path2:
                return float("inf")

            total_cost = len(path1) + len(path2)

        else:
            cost1 = self.heuristic(robot.grid_pos, rack_pos)
            cost2 = self.heuristic(rack_pos, task.goal_pos)
            total_cost = cost1 + cost2
        
        return total_cost
    
    def get_rack_access_point(self, rack_label: str) -> Tuple[int, int]:
        if rack_label in self.racks:
            return self.racks[rack_label]['access_point']
        return (0, 0)
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = pos
        neighbors = []
        # Prioritaskan gerakan, lalu wait
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < self.grid_size[0] and 
                0 <= ny < self.grid_size[1] and 
                (nx, ny) not in self.obstacles):
                neighbors.append((nx, ny))
        
        # Wait action
        neighbors.append(pos)
        return neighbors
    
    def cooperative_astar(self, start: Tuple[int, int], goal: Tuple[int, int], 
                         start_time: int = 0, robot_id: Optional[int] = None,
                         max_time: int = 400) -> List[Tuple[int, int]]:
        open_set = []
        closed_set = set()
        
        start_node = Node(
            f=self.heuristic(start, goal),
            g=0,
            h=self.heuristic(start, goal),
            pos=start,
            time=start_time
        )
        
        heapq.heappush(open_set, start_node)
        g_scores = {(start, start_time): 0}
        
        while open_set:
            current = heapq.heappop(open_set)
            
            if current.pos == goal:
                path = []
                node = current
                while node:
                    path.append(node.pos)
                    node = node.parent
                return path[::-1]
            
            state = (current.pos, current.time)
            if state in closed_set or current.time > max_time:
                continue
            
            closed_set.add(state)
            
            for neighbor in self.get_neighbors(current.pos):
                next_time = current.time + 1
                
                # Cek vertex conflict
                if self.reservation_table.check_vertex_conflict(neighbor, next_time, robot_id):
                    continue
                
                # Cek edge conflict (swap)
                if neighbor != current.pos:
                    if self.reservation_table.check_edge_conflict(current.pos, neighbor, 
                                                                  current.time, robot_id):
                        continue
                
                neighbor_state = (neighbor, next_time)
                if neighbor_state in closed_set:
                    continue
                
                # Cost lebih tinggi untuk wait
                move_cost = 1.0 if neighbor == current.pos else 1.0
                tentative_g = current.g + move_cost
                
                if neighbor_state in g_scores and tentative_g >= g_scores[neighbor_state]:
                    continue
                
                g_scores[neighbor_state] = tentative_g
                h = self.heuristic(neighbor, goal)
                f = tentative_g + h
                
                neighbor_node = Node(
                    f=f,
                    g=tentative_g,
                    h=h,
                    pos=neighbor,
                    time=next_time,
                    parent=current
                )
                
                heapq.heappush(open_set, neighbor_node)
        
        return []
    
    def plan_robot_path(self, robot: Robot):
        task = robot.current_task
        if task is None:
            return
        
        self.reservation_table.remove_robot_reservations(robot.robot_id)
        
        robot.rack_pos = self.get_rack_access_point(task.cargo_type)
        robot.path_start_time = 0
        robot.current_time_step = 0
        
        if task.mode == "store":
            path1 = self.cooperative_astar(robot.grid_pos, task.pickup_pos, 
                                          start_time=0, robot_id=robot.robot_id)
            
            if path1:
                self.add_path_to_reservations(path1, 0, robot.robot_id)
                
                start_time = len(path1) + robot.loading_time // 2
                path2 = self.cooperative_astar(task.pickup_pos, robot.rack_pos, 
                                              start_time, robot.robot_id)
                
                if path2:
                    robot.path = path1 + path2[1:]
                    self.add_path_to_reservations(path2, start_time, robot.robot_id)
                else:
                    robot.path = path1
        else:
            path1 = self.cooperative_astar(robot.grid_pos, robot.rack_pos, 
                                          start_time=0, robot_id=robot.robot_id)
            
            if path1:
                self.add_path_to_reservations(path1, 0, robot.robot_id)
                
                start_time = len(path1) + robot.loading_time // 2
                path2 = self.cooperative_astar(robot.rack_pos, task.goal_pos, 
                                              start_time, robot.robot_id)
                
                if path2:
                    robot.path = path1 + path2[1:]
                    self.add_path_to_reservations(path2, start_time, robot.robot_id)
                else:
                    robot.path = path1
        
        robot.current_phase = "moving_to_pickup" if task.mode == "store" else "moving_to_rack"
        robot.moving = True
        
        print(f"Robot {robot.robot_id + 1} planned path with {len(robot.path)} steps")
    
    def add_path_to_reservations(self, path: List[Tuple[int, int]], start_time: int, robot_id: int):
        for i, pos in enumerate(path):
            self.reservation_table.add_reservation(pos, start_time + i, robot_id)
        
        if path:
            for t in range(len(path), len(path) + 30):
                self.reservation_table.add_reservation(path[-1], start_time + t, robot_id)
    
    def check_collision(self) -> bool:
        """Deteksi collision antar robot"""
        for i, robot1 in enumerate(self.robots):
            for j, robot2 in enumerate(self.robots):
                if i >= j:
                    continue
                
                dist = ((robot1.robot_x - robot2.robot_x)**2 + 
                       (robot1.robot_y - robot2.robot_y)**2)**0.5
                
                if dist < 0.75:  # Threshold collision
                    print(f"WARNING: Collision detected between Robot {i+1} and Robot {j+1}")
                    return True
        return False
    
    def update_robots(self):
        all_done = True
        
        for robot in self.robots:
            self.reservation_table.add_reservation(
                robot.grid_pos,
                robot.current_time_step,
                robot.robot_id
            )
            if robot.current_task is None:
                continue
            

            if robot.current_phase in ["loading", "unloading"] or robot.wait_counter > 0:
                self.reservation_table.add_reservation(
                    robot.grid_pos,
                    robot.current_time_step,
                    robot.robot_id
                )

            
            if not robot.reached_goal:
                all_done = False
                
                if robot.wait_counter > 0:
                    robot.wait_counter -= 1
                    robot.wait_in_place()
                    continue
                
                if robot.moving:
                    robot.smooth_move()
                
                task = robot.current_task
                current_pos = robot.grid_pos  # Gunakan posisi diskrit
                
                if robot.current_phase in ["moving_to_pickup", "moving_to_rack"]:
                    if task.mode == "store":
                        if current_pos == task.pickup_pos:
                            robot.moving = False
                            robot.current_phase = "loading"
                            robot.wait_counter = robot.loading_time
                            print(f"Robot {robot.robot_id + 1} LOADING cargo {task.cargo_type} di pickup...")
                    else:
                        if current_pos == robot.rack_pos:
                            robot.moving = False
                            robot.current_phase = "loading"
                            robot.wait_counter = robot.loading_time
                            print(f"Robot {robot.robot_id + 1} LOADING cargo {task.cargo_type} dari rak...")
                
                elif robot.current_phase == "loading":
                    if robot.wait_counter == 0:
                        robot.has_cargo = True
                        robot.moving = True
                        robot.current_phase = "transporting"
                        print(f"Robot {robot.robot_id + 1} SELESAI loading, mulai transport")
                
                elif robot.current_phase == "transporting":
                    if task.mode == "store":
                        if current_pos == robot.rack_pos:
                            robot.moving = False
                            robot.current_phase = "unloading"
                            robot.wait_counter = robot.unloading_time
                            print(f"Robot {robot.robot_id + 1} UNLOADING cargo {task.cargo_type} di rak...")
                    else:
                        if current_pos == task.goal_pos:
                            robot.moving = False
                            robot.current_phase = "unloading"
                            robot.wait_counter = robot.unloading_time
                            print(f"Robot {robot.robot_id + 1} UNLOADING cargo {task.cargo_type} di delivery...")
                
                elif robot.current_phase == "unloading":
                    if robot.wait_counter == 0:
                        robot.has_cargo = False
                        robot.reached_goal = True
                        robot.current_phase = "completed"
                        self.completed_task_ids.add(task.task_id)
                        print(f"Robot {robot.robot_id + 1} COMPLETED Task {task.task_id}!")
                        
                        if self.available_task_ids:
                            self.reassign_idle_robots()
        
        # Check collision
        self.check_collision()
        
        self.all_completed = all_done
    
    def reassign_idle_robots(self):
        idle_robots = [r for r in self.robots if r.current_phase == "completed"]
        
        if not idle_robots or not self.available_task_ids:
            return

        
        print("\n=== Re-assigning tasks to idle robots ===")
        
        all_bids: List[Bid] = []
        for robot in idle_robots:
            for task_id in self.available_task_ids:
                task = self.all_tasks[task_id]
                cost = self.calculate_bid_cost(robot, task)
                bid = Bid(robot_id=robot.robot_id, task_id=task_id, cost=cost)
                all_bids.append(bid)
        
        all_bids = all_bids[:len(idle_robots) * 2]

        
        assigned_robots = set()
        assigned_tasks = set()
        
        for bid in all_bids:
            if bid.robot_id not in assigned_robots and bid.task_id not in assigned_tasks:
                robot = self.robots[bid.robot_id]
                task = self.all_tasks[bid.task_id]
                
                robot.current_task = task
                robot.reset_for_new_task()
                task.assigned_robot = bid.robot_id
                
                assigned_robots.add(bid.robot_id)
                assigned_tasks.add(bid.task_id)
                
                print(f"Robot {robot.robot_id + 1} assigned new Task {task.task_id} "
                      f"({task.mode} {task.cargo_type}) with cost: {bid.cost:.2f}")
                
                self.plan_robot_path(robot)
        
        for task_id in assigned_tasks:
            self.available_task_ids.discard(task_id)
    
    def draw_grid(self):
        for x in range(self.grid_size[0]):
            for y in range(self.grid_size[1]):
                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(self.screen, WHITE, rect)
                pygame.draw.rect(self.screen, GRAY, rect, 1)
    
    def draw_obstacles(self):
        for rack_label, rack_info in self.racks.items():
            rack_color = rack_info['color']
            
            for pos in rack_info['positions']:
                x, y = pos
                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(self.screen, rack_color, rect)
                pygame.draw.rect(self.screen, BLACK, rect, 2)
            
            first_pos = rack_info['positions'][0]
            last_pos = rack_info['positions'][-1]
            center_x = (first_pos[0] + last_pos[0]) / 2 * CELL_SIZE + CELL_SIZE // 2
            center_y = first_pos[1] * CELL_SIZE + CELL_SIZE // 2
            
            label_bg = pygame.Rect(center_x - 12, center_y - 10, 24, 20)
            pygame.draw.rect(self.screen, WHITE, label_bg)
            pygame.draw.rect(self.screen, BLACK, label_bg, 2)
            
            text = self.small_font.render(rack_label, True, BLACK)
            text_rect = text.get_rect(center=(center_x, center_y))
            self.screen.blit(text, text_rect)
    
    def draw_pickup_delivery_points(self):
        for i, pickup in enumerate(self.pickup_points):
            px, py = pickup
            pickup_rect = pygame.Rect(px * CELL_SIZE + 5, py * CELL_SIZE + 5,
                                     CELL_SIZE - 10, CELL_SIZE - 10)
            pygame.draw.rect(self.screen, GREEN, pickup_rect)
            pygame.draw.rect(self.screen, BLACK, pickup_rect, 2)
            
            text = self.tiny_font.render(f"P{i+1}", True, BLACK)
            text_rect = text.get_rect(center=(px * CELL_SIZE + CELL_SIZE // 2,
                                             py * CELL_SIZE + CELL_SIZE // 2))
            self.screen.blit(text, text_rect)
        
        for i, delivery in enumerate(self.delivery_points):
            dx, dy = delivery
            delivery_rect = pygame.Rect(dx * CELL_SIZE + 5, dy * CELL_SIZE + 5,
                                       CELL_SIZE - 10, CELL_SIZE - 10)
            pygame.draw.rect(self.screen, RED, delivery_rect)
            pygame.draw.rect(self.screen, BLACK, delivery_rect, 2)
            
            text = self.tiny_font.render(f"D{i+1}", True, WHITE)
            text_rect = text.get_rect(center=(dx * CELL_SIZE + CELL_SIZE // 2,
                                             dy * CELL_SIZE + CELL_SIZE // 2))
            self.screen.blit(text, text_rect)
    
    def draw_paths(self):
        for robot in self.robots:
            if robot.path and len(robot.path) > 1 and not robot.reached_goal:
                points = []
                for pos in robot.path:
                    px = pos[0] * CELL_SIZE + CELL_SIZE // 2
                    py = pos[1] * CELL_SIZE + CELL_SIZE // 2
                    points.append((px, py))
                
                if len(points) > 1:
                    color = tuple(min(255, c + 80) for c in robot.color)
                    pygame.draw.lines(self.screen, color, False, points, 2)
    
    def draw_robots(self):
        for robot in self.robots:
            x = robot.robot_x * CELL_SIZE + CELL_SIZE // 2
            y = robot.robot_y * CELL_SIZE + CELL_SIZE // 2
            
            pygame.draw.circle(self.screen, robot.color, (int(x), int(y)), 18)
            pygame.draw.circle(self.screen, WHITE, (int(x), int(y)), 18, 2)
            
            id_text = self.tiny_font.render(str(robot.robot_id + 1), True, WHITE)
            id_rect = id_text.get_rect(center=(int(x), int(y)))
            self.screen.blit(id_text, id_rect)
            
            # Visualisasi wait
            if robot.moving and robot.current_step < len(robot.path):
                next_pos = robot.path[robot.current_step]
                if next_pos == robot.grid_pos:  # Robot sedang wait
                    wait_circle = pygame.Rect(int(x) - 8, int(y) - 30, 16, 16)
                    pygame.draw.circle(self.screen, YELLOW, (int(x), int(y) - 22), 8)
                    pygame.draw.circle(self.screen, BLACK, (int(x), int(y) - 22), 8, 2)
                    wait_text = self.tiny_font.render("W", True, BLACK)
                    wait_rect = wait_text.get_rect(center=(int(x), int(y) - 22))
                    self.screen.blit(wait_text, wait_rect)
            
            if robot.has_cargo and robot.current_task:
                task = robot.current_task
                cargo_color = self.racks[task.cargo_type]['color'] if task.cargo_type in self.racks else YELLOW
                cargo_rect = pygame.Rect(int(x) - 8, int(y) - 26, 16, 12)
                pygame.draw.rect(self.screen, cargo_color, cargo_rect)
                pygame.draw.rect(self.screen, BLACK, cargo_rect, 1)
                
                text = self.tiny_font.render(task.cargo_type, True, BLACK)
                text_rect = text.get_rect(center=(int(x), int(y) - 20))
                self.screen.blit(text, text_rect)
            
            if robot.current_phase in ["loading", "unloading"]:
                status_text = "⬇" if robot.current_phase == "loading" else "⬆"
                
                progress = 1 - (robot.wait_counter / robot.loading_time if robot.current_phase == "loading" 
                            else robot.wait_counter / robot.unloading_time)
                
                bar_width = 30
                bar_height = 4
                bar_x = int(x) - bar_width // 2
                bar_y = int(y) + 25
                
                pygame.draw.rect(self.screen, GRAY, (bar_x, bar_y, bar_width, bar_height))
                pygame.draw.rect(self.screen, GREEN, (bar_x, bar_y, int(bar_width * progress), bar_height))
                pygame.draw.rect(self.screen, BLACK, (bar_x, bar_y, bar_width, bar_height), 1)
                
                status_surface = self.tiny_font.render(status_text, True, BLACK)
                self.screen.blit(status_surface, (int(x) - 5, bar_y - 15))
    
    def draw_stats(self):
        stats_y = 10
        total_tasks = len(self.all_tasks)
        in_progress = len([r for r in self.robots if r.current_task and not r.reached_goal])
        
        stats_text = [
            f"Completed: {len(self.completed_task_ids)}/{total_tasks}",
            f"In Progress: {in_progress}",
            f"Available: {len(self.available_task_ids)}"
        ]
        
        for i, text in enumerate(stats_text):
            surface = self.small_font.render(text, True, BLACK)
            bg_rect = pygame.Rect(10, stats_y + i * 25, 200, 22)
            pygame.draw.rect(self.screen, WHITE, bg_rect)
            pygame.draw.rect(self.screen, BLACK, bg_rect, 1)
            self.screen.blit(surface, (15, stats_y + i * 25 + 3))
    
    def run(self):
        while self.running:
            self.clock.tick(60)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
            
            self.update_robots()
            
            self.screen.fill(WHITE)
            self.draw_grid()
            self.draw_obstacles()
            self.draw_pickup_delivery_points()
            self.draw_paths()
            self.draw_robots()
            self.draw_stats()
            
            pygame.display.flip()
        
        pygame.quit()

def main():
    system = CooperativeAStarSystem(num_robots=5)
    system.run()

if __name__ == "__main__":
    main()