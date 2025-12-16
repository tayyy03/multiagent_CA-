import pygame
from typing import List
import config

class WarehouseGUI:
    def __init__(self, robots: List):
        pygame.init()
        self.robots = robots
        
        self.screen = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
        pygame.display.set_caption("Warehouse Multi-Robot System - Cooperative A*")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)
        self.tiny_font = pygame.font.Font(None, 18)
        
        self.running = True
    
    def draw_grid(self):
        for x in range(config.GRID_WIDTH):
            for y in range(config.GRID_HEIGHT):
                rect = pygame.Rect(x * config.CELL_SIZE, y * config.CELL_SIZE, 
                                  config.CELL_SIZE, config.CELL_SIZE)
                pygame.draw.rect(self.screen, config.WHITE, rect)
                pygame.draw.rect(self.screen, config.GRAY, rect, 1)
    
    def draw_racks(self):
        for rack_label, rack_info in config.RACKS.items():
            for pos in rack_info['positions']:
                x, y = pos
                rect = pygame.Rect(x * config.CELL_SIZE, y * config.CELL_SIZE, 
                                  config.CELL_SIZE, config.CELL_SIZE)
                pygame.draw.rect(self.screen, rack_info['color'], rect)
                pygame.draw.rect(self.screen, config.BLACK, rect, 2)
            
            first_pos = rack_info['positions'][0]
            last_pos = rack_info['positions'][-1]
            center_x = (first_pos[0] + last_pos[0]) / 2 * config.CELL_SIZE + config.CELL_SIZE // 2
            center_y = first_pos[1] * config.CELL_SIZE + config.CELL_SIZE // 2
            
            label_bg = pygame.Rect(center_x - 15, center_y - 12, 30, 24)
            pygame.draw.rect(self.screen, config.WHITE, label_bg)
            pygame.draw.rect(self.screen, config.BLACK, label_bg, 2)
            
            text = self.small_font.render(rack_label, True, config.BLACK)
            text_rect = text.get_rect(center=(center_x, center_y))
            self.screen.blit(text, text_rect)
    
    def draw_kasir_points(self):
        for i, pickup in enumerate(config.PICKUP_POINTS):
            px, py = pickup
            rect = pygame.Rect(px * config.CELL_SIZE + 5, py * config.CELL_SIZE + 5,
                              config.CELL_SIZE - 10, config.CELL_SIZE - 10)
            pygame.draw.rect(self.screen, config.GREEN, rect)
            pygame.draw.rect(self.screen, config.BLACK, rect, 2)
            
            text = self.tiny_font.render(f"P{i+1}", True, config.BLACK)
            text_rect = text.get_rect(center=(px * config.CELL_SIZE + config.CELL_SIZE // 2,
                                             py * config.CELL_SIZE + config.CELL_SIZE // 2))
            self.screen.blit(text, text_rect)
        
        for i, delivery in enumerate(config.DELIVERY_POINTS):
            dx, dy = delivery
            rect = pygame.Rect(dx * config.CELL_SIZE + 5, dy * config.CELL_SIZE + 5,
                              config.CELL_SIZE - 10, config.CELL_SIZE - 10)
            pygame.draw.rect(self.screen, config.RED, rect)
            pygame.draw.rect(self.screen, config.BLACK, rect, 2)
            
            text = self.tiny_font.render(f"D{i+1}", True, config.WHITE)
            text_rect = text.get_rect(center=(dx * config.CELL_SIZE + config.CELL_SIZE // 2,
                                             dy * config.CELL_SIZE + config.CELL_SIZE // 2))
            self.screen.blit(text, text_rect)
        
        # Home positions
        for i, home_pos in enumerate(config.ROBOT_START_POSITIONS):
            sx, sy = home_pos
            start_rect = pygame.Rect(sx * config.CELL_SIZE + 8, sy * config.CELL_SIZE + 8,
                                    config.CELL_SIZE - 16, config.CELL_SIZE - 16)
            
            robot_color = config.ROBOT_COLORS[i]
            pygame.draw.rect(self.screen, robot_color, start_rect, 2)
    
    def draw_paths(self):
        for robot in self.robots:
            if robot.path and len(robot.path) > 1:
                points = []
                for pos in robot.path[robot.current_step:]:
                    px = pos[0] * config.CELL_SIZE + config.CELL_SIZE // 2
                    py = pos[1] * config.CELL_SIZE + config.CELL_SIZE // 2
                    points.append((px, py))
                
                if len(points) > 1:
                    robot_color = config.ROBOT_COLORS[robot.robot_index]
                    pygame.draw.lines(self.screen, robot_color, False, points, 2)
    
    def draw_robots(self):
        for robot in self.robots:
            x = robot.robot_x * config.CELL_SIZE + config.CELL_SIZE // 2
            y = robot.robot_y * config.CELL_SIZE + config.CELL_SIZE // 2
            
            robot_color = config.ROBOT_COLORS[robot.robot_index]
            
            pygame.draw.circle(self.screen, robot_color, (int(x), int(y)), 18)
            pygame.draw.circle(self.screen, config.WHITE, (int(x), int(y)), 18, 2)
            
            # Robot ID
            id_text = self.tiny_font.render(str(robot.robot_index + 1), True, config.WHITE)
            id_rect = id_text.get_rect(center=(int(x), int(y)))
            self.screen.blit(id_text, id_rect)
            
            if robot.has_cargo and robot.cargo_type:
                cargo_color = config.RACKS[robot.cargo_type]['color']
                cargo_rect = pygame.Rect(int(x) - 8, int(y) - 26, 16, 14)
                pygame.draw.rect(self.screen, cargo_color, cargo_rect)
                pygame.draw.rect(self.screen, config.BLACK, cargo_rect, 1)
                
                text = self.tiny_font.render(robot.cargo_type, True, config.BLACK)
                text_rect = text.get_rect(center=(int(x), int(y) - 19))
                self.screen.blit(text, text_rect)
            
            if robot.is_loading:
                bar_width = 36
                bar_height = 5
                bar_x = int(x) - bar_width // 2
                bar_y = int(y) + 22
                
                pygame.draw.rect(self.screen, config.GRAY, (bar_x, bar_y, bar_width, bar_height))
                progress_width = int(bar_width * robot.loading_progress)
                pygame.draw.rect(self.screen, config.ORANGE, (bar_x, bar_y, progress_width, bar_height))
                pygame.draw.rect(self.screen, config.BLACK, (bar_x, bar_y, bar_width, bar_height), 1)
    
    def draw_info(self):
        info_y = 10
        
        title_text = "Multi-Robot Warehouse System"
        text = self.small_font.render(title_text, True, config.BLACK)
        self.screen.blit(text, (10, info_y))
        info_y += 35
        
        for robot in self.robots:
            robot_color = config.ROBOT_COLORS[robot.robot_index]
            
            # Robot status indicator
            status_circle = pygame.Rect(10, info_y, 12, 12)
            if robot.status == "IDLE":
                pygame.draw.circle(self.screen, config.GREEN, (16, info_y + 6), 6)
            elif robot.is_returning_home:
                pygame.draw.circle(self.screen, config.ORANGE, (16, info_y + 6), 6)
            else:
                pygame.draw.circle(self.screen, config.RED, (16, info_y + 6), 6)
            
            pygame.draw.circle(self.screen, config.BLACK, (16, info_y + 6), 6, 1)
            
            queue_size = robot.task_queue.qsize()
            status_str = "RETURN" if robot.is_returning_home else robot.status
            robot_text = f"R{robot.robot_index + 1}: {status_str} | Q:{queue_size}"
            
            text = self.tiny_font.render(robot_text, True, robot_color)
            self.screen.blit(text, (30, info_y))
            info_y += 20
    
    def run(self):
        while self.running:
            self.clock.tick(60)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
            
            for robot in self.robots:
                robot.update()
            
            self.screen.fill(config.WHITE)
            self.draw_grid()
            self.draw_racks()
            self.draw_kasir_points()
            self.draw_paths()
            self.draw_robots()
            self.draw_info()
            
            pygame.display.flip()
        
        pygame.quit()