import json
import time
import random
from typing import List, Tuple, Optional
from queue import Queue
import paho.mqtt.client as mqtt
from cooperative_astar import CooperativeAStar
import config

class RobotController:
    def __init__(self, robot_id: str, robot_index: int, shared_pathfinder: CooperativeAStar):
        self.robot_id = robot_id
        self.robot_index = robot_index
        self.status = "IDLE"
        self.current_phase = None
        self.has_cargo = False
        self.cargo_type = None
        
        self.robot_pos = config.ROBOT_START_POSITIONS[robot_index]
        self.robot_x = float(self.robot_pos[0])
        self.robot_y = float(self.robot_pos[1])
        
        self.path: List[Tuple[int, int]] = []
        self.current_step = 0
        
        self.is_loading = False
        self.loading_start_time = 0
        self.loading_progress = 0
        
        self.task_queue = Queue()
        self.current_task = None
        
        self.is_returning_home = False
        self.home_position = config.ROBOT_START_POSITIONS[robot_index]
        
        self.pathfinder = shared_pathfinder
        self.current_reservation = None
        
        # CNP state
        self.pending_bids: set = set()
        
        # Waiting system untuk access control
        self.is_waiting_access = False
        self.waiting_for_location = None
        self.access_granted = False
        
        self.mqtt_client = mqtt.Client(client_id=self.robot_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        try:
            self.mqtt_client.connect(config.MQTT_BROKER, config.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"{self.robot_id} MQTT connection failed: {e}")
    
    def on_connect(self, client, userdata, flags, rc):
        print(f"{self.robot_id} connected to MQTT broker")
        client.subscribe(config.MQTT_TOPIC_TASK_ANNOUNCE)
        client.subscribe(f"{config.MQTT_TOPIC_TASK_AWARD}/{self.robot_id}")
        client.subscribe(f"{config.MQTT_TOPIC_COORDINATOR}/{self.robot_id}")
        self.publish_status()
    
    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            
            if msg.topic == config.MQTT_TOPIC_TASK_ANNOUNCE:
                self.handle_task_announcement(data)
            
            elif msg.topic == f"{config.MQTT_TOPIC_TASK_AWARD}/{self.robot_id}":
                self.handle_task_award(data)
            
            elif msg.topic == f"{config.MQTT_TOPIC_COORDINATOR}/{self.robot_id}":
                # Check apakah ini response untuk access request
                if 'access_granted' in data:
                    self.handle_access_response(data)
                else:
                    self.handle_coordinator_response(data)
                
        except Exception as e:
            print(f"{self.robot_id} error processing message: {e}")
    
    def handle_task_announcement(self, data):
        """Handle task announcement dan submit bid"""
        if 'deadline' not in data:
            return
        
        task_id = data['task_id']
        task_data = data['task_data']
        
        if task_id in self.pending_bids:
            return
        
        bid_value = self.calculate_bid_value(task_data)
        
        if bid_value > 0:
            bid = {
                'task_id': task_id,
                'robot_id': self.robot_id,
                'bid_value': bid_value,
                'timestamp': time.time()
            }
            
            self.mqtt_client.publish(config.MQTT_TOPIC_TASK_BID, json.dumps(bid))
            self.pending_bids.add(task_id)
            print(f"{self.robot_id} submitted bid for {task_id}: {bid_value:.2f}")
    
    def calculate_bid_value(self, task_data) -> float:
        """Calculate bid value untuk task"""
        task_type = task_data.get('task_type', random.choice(['STORE', 'RETRIEVE']))
        
        if task_type == 'STORE':
            target_pos = config.PICKUP_POINTS[task_data['pickup_idx']]
        else:
            target_pos = config.RACKS[task_data['retrieve_rack']]['entry_point']
        
        distance = self.calculate_distance(self.robot_pos, target_pos)
        max_distance = config.GRID_WIDTH + config.GRID_HEIGHT
        distance_score = (max_distance - distance) / max_distance * 100
        
        status_bonus = 0
        if self.status == "IDLE" and not self.is_returning_home:
            status_bonus = config.CNP_IDLE_PRIORITY_BONUS
        elif self.is_returning_home:
            status_bonus = 50
        
        queue_penalty = self.task_queue.qsize() * 20
        
        bid_value = distance_score + status_bonus - queue_penalty
        
        return max(bid_value, 0)
    
    def calculate_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def handle_task_award(self, data):
        """Handle task yang di-award ke robot ini"""
        task_id = data['task_id']
        task_data = data['task_data']
        
        print(f"\n{self.robot_id} awarded task {task_id}")
        if 'reason' in data:
            print(f"  Reason: {data['reason']}")
        elif 'bid_value' in data:
            print(f"  Winning bid: {data['bid_value']:.2f}")
        
        self.pending_bids.discard(task_id)
        self.task_queue.put(task_data)
        
        if self.is_returning_home:
            print(f"  {self.robot_id} cancelling return home")
            self.is_returning_home = False
            self.status = "IDLE"
            self.current_phase = None
            self.path = []
            self.current_step = 0
    
    def handle_access_response(self, data):
        """Handle response access dari coordinator"""
        location = tuple(data['location'])
        
        if data['access_granted']:
            print(f"{self.robot_id} access GRANTED to {location}")
            self.is_waiting_access = False
            self.access_granted = True
        else:
            queue_pos = data.get('queue_position', 0)
            print(f"{self.robot_id} WAITING for {location} (queue pos: {queue_pos})")
            self.is_waiting_access = True
            self.access_granted = False
    
    def handle_coordinator_response(self, data):
        """Handle response dari coordinator"""
        if 'approved' in data:
            if not data['approved']:
                wait_time = data.get('waiting_time', 0)
                if wait_time > 0:
                    print(f"{self.robot_id} waiting {wait_time:.1f}s for location")
    
    def request_access(self, location: Tuple[int, int], action: str):
        """Request akses ke kasir/rak"""
        request = {
            'type': 'request_access',
            'robot_id': self.robot_id,
            'location': location,
            'action': action,
            'timestamp': time.time()
        }
        self.mqtt_client.publish(config.MQTT_TOPIC_ROBOT_RESERVATION, json.dumps(request))
        self.is_waiting_access = True
        self.waiting_for_location = location
        self.access_granted = False
        
        print(f"{self.robot_id} requesting access to {location} for {action}")
    
    def release_access(self, location: Tuple[int, int]):
        """Release akses dari kasir/rak"""
        release = {
            'type': 'release_access',
            'robot_id': self.robot_id,
            'location': location,
            'timestamp': time.time()
        }
        self.mqtt_client.publish(config.MQTT_TOPIC_ROBOT_RESERVATION, json.dumps(release))
        self.is_waiting_access = False
        self.waiting_for_location = None
        
        print(f"{self.robot_id} released access to {location}")
    
    def request_location_reservation(self, location: Tuple[int, int], action: str):
        request = {
            'type': 'request_reservation',
            'robot_id': self.robot_id,
            'location': location,
            'action': action,
            'duration': config.LOADING_TIME,
            'timestamp': time.time()
        }
        self.mqtt_client.publish(config.MQTT_TOPIC_ROBOT_RESERVATION, json.dumps(request))
        self.current_reservation = location
    
    def release_location_reservation(self, location: Tuple[int, int]):
        release = {
            'type': 'release_reservation',
            'robot_id': self.robot_id,
            'location': location,
            'timestamp': time.time()
        }
        self.mqtt_client.publish(config.MQTT_TOPIC_ROBOT_RESERVATION, json.dumps(release))
        self.current_reservation = None
    
    def publish_status(self):
        status_data = {
            "robot_id": self.robot_id,
            "status": self.status,
            "position": self.robot_pos,
            "has_cargo": self.has_cargo,
            "cargo_type": self.cargo_type,
            "current_phase": self.current_phase,
            "queue_size": self.task_queue.qsize(),
            "is_returning_home": self.is_returning_home
        }
        self.mqtt_client.publish(config.MQTT_TOPIC_ROBOT_STATUS, json.dumps(status_data))
        
        position_data = {
            "robot_id": self.robot_id,
            "position": self.robot_pos,
            "path": self.path,
            "timestamp": time.time()
        }
        self.mqtt_client.publish(config.MQTT_TOPIC_ROBOT_POSITION, json.dumps(position_data))
    
    def publish_task_complete(self, task_id: str):
        complete_data = {
            "robot_id": self.robot_id,
            "task_id": task_id,
            "timestamp": time.time()
        }
        self.mqtt_client.publish(config.MQTT_TOPIC_TASK_COMPLETE, json.dumps(complete_data))
    
    def is_critical_location(self, pos: Tuple[int, int]) -> bool:
        """Check apakah posisi adalah kasir atau entry point rak"""
        if pos in config.PICKUP_POINTS or pos in config.DELIVERY_POINTS:
            return True
        
        for rack in config.RACKS.values():
            if pos == rack['entry_point']:
                return True
        
        return False
    
    def start_task(self, task):
        self.current_task = task
        self.status = "BUSY"
        self.is_returning_home = False
        
        if 'task_type' not in task:
            task['task_type'] = random.choice(['STORE', 'RETRIEVE'])
        
        if task['task_type'] == 'STORE':
            self.current_phase = "to_pickup"
            pickup_pos = config.PICKUP_POINTS[task['pickup_idx']]
            target = self.pathfinder.find_side_position(pickup_pos)
        else:
            self.current_phase = "to_rack"
            rack_entry = config.RACKS[task['retrieve_rack']]['entry_point']
            target = rack_entry
        
        self._calculate_and_reserve_path(target)
        
        if self.path:
            print(f"{self.robot_id} starting {task['task_type']} task {task['task_id']}")
        else:
            print(f"{self.robot_id} failed to find path - task {task['task_id']}")
        
        self.publish_status()
    
    def start_loading(self):
        self.is_loading = True
        self.loading_start_time = time.time()
        self.loading_progress = 0
        
        if self.current_phase:
            self.request_location_reservation(self.robot_pos, self.current_phase)
    
    def update_loading(self):
        if self.is_loading:
            elapsed = time.time() - self.loading_start_time
            self.loading_progress = min(elapsed / config.LOADING_TIME, 1.0)
            
            if self.loading_progress >= 1.0:
                self.is_loading = False
                if self.current_reservation:
                    self.release_location_reservation(self.current_reservation)
                
                # Release access setelah loading selesai
                if self.access_granted and self.waiting_for_location:
                    self.release_access(self.waiting_for_location)
                    self.access_granted = False
                
                return True
        return False
    
    def smooth_move(self):
        if self.is_loading:
            if self.update_loading():
                self._handle_phase_transition()
            return False
        
        # Check apakah sedang waiting for access
        if self.is_waiting_access and not self.access_granted:
            # Robot menunggu, tidak bergerak
            return False
        
        # Check apakah perlu request access untuk next position
        if self.path and self.current_step < len(self.path):
            current_pos = self.path[self.current_step]
            
            # Check apakah current position adalah critical location dan belum punya akses
            if self.is_critical_location(current_pos):
                if not self.access_granted and not self.is_waiting_access:
                    # Request access
                    action = self.current_phase or "moving"
                    self.request_access(current_pos, action)
                    return False  # Wait untuk access
                elif not self.access_granted:
                    # Masih waiting
                    return False
        
        if self.current_step < len(self.path):
            target_x, target_y = self.path[self.current_step]
            
            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
            
            distance = (dx**2 + dy**2)**0.5
            
            if distance < 0.05:
                self.robot_x = float(target_x)
                self.robot_y = float(target_y)
                self.robot_pos = (target_x, target_y)
                self.current_step += 1
                
                # Release access jika sudah melewati critical location
                if self.access_granted and self.waiting_for_location:
                    # Check apakah sudah meninggalkan critical location
                    if not self.is_critical_location(self.robot_pos):
                        self.release_access(self.waiting_for_location)
                        self.access_granted = False
                
                if self.current_step >= len(self.path):
                    self.start_loading()
                    return True
            else:
                self.robot_x += dx * config.ROBOT_SPEED
                self.robot_y += dy * config.ROBOT_SPEED
                self.robot_pos = (int(round(self.robot_x)), int(round(self.robot_y)))
        
        return False
    
    def _handle_phase_transition(self):
        if not self.current_task:
            print(f"{self.robot_id} ERROR: phase transition without task")
            self.status = "IDLE"
            self.current_phase = None
            return
        
        task_type = self.current_task.get('task_type', 'STORE')
        
        if task_type == 'STORE':
            if self.current_phase == "to_pickup":
                self.has_cargo = True
                self.cargo_type = self.current_task['store_rack']
                self.current_phase = "to_rack"
                
                rack_entry = config.RACKS[self.current_task['store_rack']]['entry_point']
                self._calculate_and_reserve_path(rack_entry)
                
            elif self.current_phase == "to_rack":
                self.has_cargo = False
                self.cargo_type = None
                self._complete_task()
        
        else:  # RETRIEVE
            if self.current_phase == "to_rack":
                self.has_cargo = True
                self.cargo_type = self.current_task['retrieve_rack']
                self.current_phase = "to_delivery"
                
                delivery_pos = config.DELIVERY_POINTS[self.current_task['delivery_idx']]
                target = self.pathfinder.find_side_position(delivery_pos)
                self._calculate_and_reserve_path(target)
                
            elif self.current_phase == "to_delivery":
                self.has_cargo = False
                self.cargo_type = None
                self._complete_task()
        
        self.publish_status()
    
    def _calculate_and_reserve_path(self, target: Tuple[int, int]):
        """Calculate path dengan retry logic"""
        self.pathfinder.clear_reservation(self.robot_id)
        
        for attempt in range(3):
            time_offset = attempt * 10 + self.robot_index * 3
            
            self.path = self.pathfinder.astar(
                self.robot_pos, 
                target, 
                self.robot_id, 
                start_time=time_offset
            )
            
            if self.path:
                self.pathfinder.reserve_path(self.path, self.robot_id, start_time=time_offset)
                self.current_step = 0
                return
            
            time.sleep(0.05 * (attempt + 1))
        
        print(f"{self.robot_id} WARNING: No path found to {target}")
        self.path = []
        self.current_step = 0
    
    def _complete_task(self):
        """Handle task completion"""
        if not self.current_task:
            return
        
        print(f"\n{self.robot_id} completed task {self.current_task['task_id']}")
        self.publish_task_complete(self.current_task['task_id'])
        self.current_task = None
        
        if not self.task_queue.empty():
            print(f"{self.robot_id} has {self.task_queue.qsize()} task(s) waiting")
            self.status = "IDLE"
            self.current_phase = None
        else:
            print(f"{self.robot_id} returning home")
            self.current_phase = "return_home"
            self.is_returning_home = True
            self._calculate_and_reserve_path(self.home_position)
    
    def _handle_return_home(self):
        """Handle arrival at home"""
        print(f"{self.robot_id} arrived home - IDLE")
        self.status = "IDLE"
        self.current_phase = None
        self.is_returning_home = False
        self.pathfinder.clear_reservation(self.robot_id)
        self.publish_status()
    
    def update(self):
        if self.status == "IDLE" and not self.task_queue.empty() and not self.is_returning_home:
            task = self.task_queue.get()
            self.start_task(task)
        
        if self.status == "BUSY" and self.path:
            self.smooth_move()
        
        if self.is_returning_home and self.path:
            if self.smooth_move():
                pass
            elif self.current_step >= len(self.path) and not self.is_loading:
                self._handle_return_home()