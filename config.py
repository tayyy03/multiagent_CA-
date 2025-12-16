# Konfigurasi sistem warehouse

# MQTT Configuration
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC_TASK = "warehouse/task"
MQTT_TOPIC_ROBOT_STATUS = "warehouse/robot/status"
MQTT_TOPIC_TASK_COMPLETE = "warehouse/task/complete"
MQTT_TOPIC_ROBOT_POSITION = "warehouse/robot/position"
MQTT_TOPIC_ROBOT_RESERVATION = "warehouse/robot/reservation"
MQTT_TOPIC_COORDINATOR = "warehouse/coordinator"
MQTT_TOPIC_TASK_BID = "warehouse/task/bid"
MQTT_TOPIC_TASK_AWARD = "warehouse/task/award"
MQTT_TOPIC_TASK_ANNOUNCE = "warehouse/task/announce"

# CNP Configuration
CNP_BID_TIMEOUT = 2.0  # Waktu tunggu untuk bid (detik)
CNP_IDLE_PRIORITY_BONUS = 100 

# CA* Configuration
RESERVATION_WINDOW = 100
SAFE_DISTANCE = 1
ROBOT_SPEED = 0.06  # Turunkan sedikit untuk better collision avoidance

# Grid Configuration
CELL_SIZE = 50
GRID_WIDTH = 15
GRID_HEIGHT = 12
WINDOW_WIDTH = CELL_SIZE * GRID_WIDTH
WINDOW_HEIGHT = CELL_SIZE * GRID_HEIGHT

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 100, 255)
YELLOW = (255, 255, 0)
DARK_GRAY = (100, 100, 100)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
LIME = (50, 205, 50)
PINK = (255, 192, 203)

ROBOT_COLORS = [
    BLUE,      # Robot 1
    RED,       # Robot 2
    GREEN,     # Robot 3
    ORANGE,    # Robot 4
    PURPLE     # Robot 5
]

# Warehouse Configuration
PICKUP_POINTS = [(2, 0), (4, 0), (6, 0), (8, 0), (10, 0)]
DELIVERY_POINTS = [(2, 11), (4, 11), (6, 11), (8, 11), (10, 11)]

RACKS = {
    'A': {
        'positions': [(i, 3) for i in range(1, 6)],
        'color': (255, 100, 100),
        'entry_point': (3, 2)
    },
    'B': {
        'positions': [(i, 5) for i in range(1, 6)],
        'color': (100, 100, 255),
        'entry_point': (3, 4)
    },
    'C': {
        'positions': [(i, 7) for i in range(1, 6)],
        'color': (100, 255, 100),
        'entry_point': (3, 6)
    },
    'D': {
        'positions': [(i, 9) for i in range(1, 6)],
        'color': (255, 255, 100),
        'entry_point': (3, 8)
    },
    'E': {
        'positions': [(i, 3) for i in range(8, 13)],
        'color': (255, 150, 255),
        'entry_point': (10, 2)
    },
    'F': {
        'positions': [(i, 5) for i in range(8, 13)],
        'color': (150, 255, 255),
        'entry_point': (10, 4)
    },
    'G': {
        'positions': [(i, 7) for i in range(8, 13)],
        'color': (255, 200, 150),
        'entry_point': (10, 6)
    },
    'H': {
        'positions': [(i, 9) for i in range(8, 13)],
        'color': (200, 150, 255),
        'entry_point': (10, 8)
    }
}

# Robot Configuration
ROBOT_START_POSITIONS = [
    (14, 4),   # Robot 1
    (14, 5),   # Robot 2
    (14, 6),   # Robot 3
    (14, 7),   # Robot 4
    (14, 8)    # Robot 5
]

ROBOT_SPEED = 0.08
LOADING_TIME = 2.0
NUM_ROBOTS = 5

# CA* Configuration
RESERVATION_WINDOW = 100  # Timesteps untuk reservasi
SAFE_DISTANCE = 1  # Jarak minimum antar robot