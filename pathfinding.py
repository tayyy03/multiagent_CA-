import heapq
from typing import List, Tuple, Set, Optional
from dataclasses import dataclass, field

@dataclass(order=True)
class Node:
    f: float
    g: float = field(compare=False)
    h: float = field(compare=False)
    pos: Tuple[int, int] = field(compare=False)
    parent: Optional['Node'] = field(default=None, compare=False)

class PathFinder:
    def __init__(self, grid_size: Tuple[int, int], obstacles: Set[Tuple[int, int]]):
        self.grid_size = grid_size
        self.obstacles = obstacles
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = pos
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < self.grid_size[0] and 
                0 <= ny < self.grid_size[1] and 
                (nx, ny) not in self.obstacles):
                neighbors.append((nx, ny))
        return neighbors
    
    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        open_set = []
        closed_set = set()
        
        start_node = Node(
            f=self.heuristic(start, goal),
            g=0,
            h=self.heuristic(start, goal),
            pos=start
        )
        
        heapq.heappush(open_set, start_node)
        g_scores = {start: 0}
        
        while open_set:
            current = heapq.heappop(open_set)
            
            if current.pos == goal:
                path = []
                node = current
                while node:
                    path.append(node.pos)
                    node = node.parent
                return path[::-1]
            
            if current.pos in closed_set:
                continue
                
            closed_set.add(current.pos)
            
            for neighbor in self.get_neighbors(current.pos):
                if neighbor in closed_set:
                    continue
                
                tentative_g = current.g + 1
                
                if neighbor in g_scores and tentative_g >= g_scores[neighbor]:
                    continue
                
                g_scores[neighbor] = tentative_g
                h = self.heuristic(neighbor, goal)
                f = tentative_g + h
                
                neighbor_node = Node(
                    f=f,
                    g=tentative_g,
                    h=h,
                    pos=neighbor,
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