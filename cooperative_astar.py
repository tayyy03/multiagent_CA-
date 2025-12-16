import heapq
from typing import List, Tuple, Set, Optional, Dict
from dataclasses import dataclass, field

@dataclass(order=True)
class Node:
    f: float
    g: float = field(compare=False)
    h: float = field(compare=False)
    pos: Tuple[int, int] = field(compare=False)
    time: int = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)

class CooperativeAStar:
    def __init__(self, grid_size: Tuple[int, int], obstacles: Set[Tuple[int, int]]):
        self.grid_size = grid_size
        self.obstacles = obstacles
        self.reservations: Dict[Tuple[int, int, int], str] = {}
        self.robot_paths: Dict[str, List[Tuple[int, int]]] = {}
        
        # Corridor detection (jalur sempit)
        self.corridors = self._detect_corridors()
    
    def _detect_corridors(self) -> Set[Tuple[int, int]]:
        """Deteksi posisi-posisi yang merupakan corridor (jalur sempit)"""
        corridors = set()
        
        for x in range(self.grid_size[0]):
            for y in range(self.grid_size[1]):
                if (x, y) in self.obstacles:
                    continue
                
                # Hitung jumlah neighbor yang bukan obstacle
                free_neighbors = 0
                for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    nx, ny = x + dx, y + dy
                    if (0 <= nx < self.grid_size[0] and 
                        0 <= ny < self.grid_size[1] and 
                        (nx, ny) not in self.obstacles):
                        free_neighbors += 1
                
                # Jika hanya 2 free neighbors (jalur lurus), ini adalah corridor
                if free_neighbors == 2:
                    corridors.add((x, y))
        
        return corridors
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = pos
        neighbors = []
        for dx, dy in [(1, 0), (0, 1), (0, -1), (-1, 0), (0, 0)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < self.grid_size[0] and 
                0 <= ny < self.grid_size[1] and 
                (nx, ny) not in self.obstacles):
                neighbors.append((nx, ny))
        return neighbors
    
    def is_in_corridor(self, pos: Tuple[int, int]) -> bool:
        """Check apakah posisi berada di corridor"""
        return pos in self.corridors
    
    def has_conflict(self, pos: Tuple[int, int], prev_pos: Tuple[int, int], 
                    time: int, robot_id: str) -> bool:
        """Enhanced conflict detection untuk corridor"""
        # Basic vertex conflict
        key = (pos[0], pos[1], time)
        if key in self.reservations and self.reservations[key] != robot_id:
            return True
        
        # Edge conflict (swap)
        if prev_pos != pos:
            prev_key = (pos[0], pos[1], time - 1)
            curr_key = (prev_pos[0], prev_pos[1], time)
            if (prev_key in self.reservations and curr_key in self.reservations):
                other = self.reservations[prev_key]
                if other != robot_id and self.reservations[curr_key] == other:
                    return True
        
        # Corridor safety: check wider time window
        if self.is_in_corridor(pos):
            for t_offset in [-2, -1, 1, 2]:
                check_key = (pos[0], pos[1], time + t_offset)
                if check_key in self.reservations and self.reservations[check_key] != robot_id:
                    return True
        
        return False
    
    def reserve_path(self, path: List[Tuple[int, int]], robot_id: str, start_time: int = 0):
        """Reserve path dengan extended reservation untuk corridor"""
        self.clear_reservation(robot_id)
        
        for t, pos in enumerate(path):
            time_step = start_time + t
            key = (pos[0], pos[1], time_step)
            self.reservations[key] = robot_id
            
            # Extended reservation untuk corridor
            if self.is_in_corridor(pos):
                # Reserve ±1 timestep di corridor untuk safety
                for offset in [-1, 1]:
                    extended_key = (pos[0], pos[1], time_step + offset)
                    if extended_key not in self.reservations:
                        self.reservations[extended_key] = robot_id
        
        # Reserve final position
        if path:
            final_pos = path[-1]
            final_time = start_time + len(path) - 1
            for extra in range(1, 10):
                key = (final_pos[0], final_pos[1], final_time + extra)
                self.reservations[key] = robot_id
        
        self.robot_paths[robot_id] = path
    
    def clear_reservation(self, robot_id: str):
        keys_to_remove = [k for k, v in self.reservations.items() if v == robot_id]
        for key in keys_to_remove:
            del self.reservations[key]
        
        if robot_id in self.robot_paths:
            del self.robot_paths[robot_id]
    
    def astar(self, start: Tuple[int, int], goal: Tuple[int, int], 
             robot_id: str, start_time: int = 0, max_iterations: int = 5000) -> List[Tuple[int, int]]:
        
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
        
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(open_set)
            
            if current.pos == goal:
                path = []
                node = current
                while node:
                    path.append(node.pos)
                    node = node.parent
                return path[::-1]
            
            state = (current.pos, current.time)
            if state in closed_set:
                continue
            
            closed_set.add(state)
            
            for neighbor in self.get_neighbors(current.pos):
                next_time = current.time + 1
                
                if self.has_conflict(neighbor, current.pos, next_time, robot_id):
                    continue
                
                tentative_g = current.g + 1
                
                # Wait penalty
                if neighbor == current.pos:
                    tentative_g += 0.5
                
                # Corridor penalty untuk encourage alternative routes
                if self.is_in_corridor(neighbor):
                    tentative_g += 0.1
                
                neighbor_state = (neighbor, next_time)
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
    
    def find_side_position(self, target_pos: Tuple[int, int]) -> Tuple[int, int]:
        x, y = target_pos
        candidates = [(x-1, y), (x+1, y), (x, y+1), (x, y-1)]
        
        for pos in candidates:
            if (0 <= pos[0] < self.grid_size[0] and 
                0 <= pos[1] < self.grid_size[1] and 
                pos not in self.obstacles):
                return pos
        
        return (x+1, y) if x+1 < self.grid_size[0] else (x-1, y)