import time
import threading
from robot import RobotController
from kasir import KasirTerminal
from arena import WarehouseGUI
from robot_coordinator import RobotCoordinator
from cooperative_astar import CooperativeAStar
import config

def create_obstacles():
    """Create obstacle set"""
    obstacles = set()
    for rack_info in config.RACKS.values():
        for pos in rack_info['positions']:
            obstacles.add(pos)
    for pickup in config.PICKUP_POINTS:
        obstacles.add(pickup)
    for delivery in config.DELIVERY_POINTS:
        obstacles.add(delivery)
    return obstacles

def kasir_simulator(kasir: KasirTerminal, num_tasks: int = 5, delay: float = 3.0):
    """Simulasi kasir mengirim task random"""
    time.sleep(3)
    
    for i in range(num_tasks):
        kasir.send_task()  # Random task type
        time.sleep(delay)

def main():
    print("="*70)
    print("Multi-Robot Warehouse System with Cooperative A*")
    print("="*70)
    print(f"Number of Robots: {config.NUM_ROBOTS}")
    print("Features:")
    print("  - Cooperative A* pathfinding")
    print("  - Collision avoidance")
    print("  - Location reservation system")
    print("  - Random task types (STORE/RETRIEVE)")
    print("  - FIFO task queue per robot")
    print("="*70)
    print()
    
    # Initialize coordinator
    coordinator = RobotCoordinator()
    
    # Initialize shared pathfinder untuk CA*
    obstacles = create_obstacles()
    shared_pathfinder = CooperativeAStar((config.GRID_WIDTH, config.GRID_HEIGHT), obstacles)
    
    # Initialize robots
    robots = []
    for i in range(config.NUM_ROBOTS):
        robot_id = f"ROBOT_{i+1:02d}"
        robot = RobotController(robot_id, i, shared_pathfinder)
        robots.append(robot)
        time.sleep(0.1)  # Small delay untuk MQTT connection
    
    print(f"\n{config.NUM_ROBOTS} robots initialized")
    
    # Initialize kasir terminals
    kasir_terminals = []
    for i in range(3):  # 3 kasir
        kasir_id = f"KASIR_{i+1}"
        kasir = KasirTerminal(kasir_id, "MIXED")
        kasir_terminals.append(kasir)
    
    print(f"{len(kasir_terminals)} kasir terminals initialized\n")
    
    # Start kasir simulators
    kasir_threads = []
    for i, kasir in enumerate(kasir_terminals):
        # Stagger task sending
        num_tasks = 4 + i
        delay = 4.0 + i * 2.0
        
        thread = threading.Thread(
            target=kasir_simulator,
            args=(kasir, num_tasks, delay),
            daemon=True
        )
        thread.start()
        kasir_threads.append(thread)
    
    # Start GUI
    print("Starting GUI...\n")
    gui = WarehouseGUI(robots)
    gui.run()
    
    # Cleanup
    for robot in robots:
        robot.mqtt_client.loop_stop()
    for kasir in kasir_terminals:
        kasir.mqtt_client.loop_stop()
    coordinator.mqtt_client.loop_stop()
    
    print("\nSystem shutdown complete")

if __name__ == "__main__":
    main()