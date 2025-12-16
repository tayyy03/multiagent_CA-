import json
import time
from typing import Dict, List, Tuple, Set, Optional
from collections import defaultdict, deque
import paho.mqtt.client as mqtt
import config
import threading

class RobotCoordinator:
    def __init__(self):
        self.location_reservations: Dict[Tuple[int, int], List[Dict]] = defaultdict(list)
        
        # Waiting queue untuk kasir dan rak
        self.location_queues: Dict[Tuple[int, int], deque] = defaultdict(deque)
        self.location_occupied: Dict[Tuple[int, int], Optional[str]] = {}
        self.robot_waiting_for: Dict[str, Optional[Tuple[int, int]]] = {}
        
        # CNP Management
        self.pending_tasks: Dict[str, Dict] = {}
        self.task_bids: Dict[str, List[Dict]] = defaultdict(list)
        self.task_assignments: Dict[str, str] = {}
        self.assigned_tasks: Set[str] = set()
        self.announced_tasks: Set[str] = set()
        self.active_timers: Dict[str, threading.Timer] = {}
        
        # Robot state tracking
        self.robot_states: Dict[str, Dict] = {}
        self.robot_positions: Dict[str, Tuple[int, int]] = {}
        
        # Initial task assignment
        self.initial_task_counter = 0
        self.all_robots_moved = False
        
        self.mqtt_client = mqtt.Client(client_id="COORDINATOR")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        try:
            self.mqtt_client.connect(config.MQTT_BROKER, config.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"Coordinator MQTT connection failed: {e}")
    
    def on_connect(self, client, userdata, flags, rc):
        print("Coordinator connected to MQTT broker")
        client.subscribe(config.MQTT_TOPIC_ROBOT_RESERVATION)
        client.subscribe(config.MQTT_TOPIC_TASK_ANNOUNCE)
        client.subscribe(config.MQTT_TOPIC_TASK_BID)
        client.subscribe(config.MQTT_TOPIC_ROBOT_STATUS)
        client.subscribe(config.MQTT_TOPIC_ROBOT_POSITION)
    
    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            
            if msg.topic == config.MQTT_TOPIC_ROBOT_RESERVATION:
                if data['type'] == 'request_reservation':
                    self.handle_reservation_request(data)
                elif data['type'] == 'release_reservation':
                    self.handle_release_reservation(data)
                elif data['type'] == 'request_access':
                    self.handle_access_request(data)
                elif data['type'] == 'release_access':
                    self.handle_access_release(data)
            
            elif msg.topic == config.MQTT_TOPIC_TASK_ANNOUNCE:
                task_id = data.get('task_id')
                if task_id and task_id not in self.announced_tasks:
                    self.handle_task_announcement(data)
            
            elif msg.topic == config.MQTT_TOPIC_TASK_BID:
                self.handle_task_bid(data)
            
            elif msg.topic == config.MQTT_TOPIC_ROBOT_STATUS:
                self.update_robot_state(data)
            
            elif msg.topic == config.MQTT_TOPIC_ROBOT_POSITION:
                self.update_robot_position(data)
                
        except Exception as e:
            print(f"Coordinator error: {e}")
    
    def handle_access_request(self, data):
        """Handle permintaan akses ke kasir/rak dengan waiting queue"""
        robot_id = data['robot_id']
        location = tuple(data['location'])
        action = data['action']
        
        print(f"Coordinator: {robot_id} requesting access to {location} for {action}")
        
        # Check apakah lokasi sedang dipakai
        if location in self.location_occupied and self.location_occupied[location]:
            # Lokasi sedang dipakai, masukkan ke queue
            if robot_id not in self.location_queues[location]:
                self.location_queues[location].append(robot_id)
                self.robot_waiting_for[robot_id] = location
                
                queue_position = len(self.location_queues[location])
                print(f"Coordinator: {robot_id} queued at {location} (position: {queue_position})")
                
                response = {
                    'robot_id': robot_id,
                    'location': location,
                    'access_granted': False,
                    'queue_position': queue_position,
                    'occupied_by': self.location_occupied[location]
                }
        else:
            # Lokasi kosong, grant akses
            self.location_occupied[location] = robot_id
            self.robot_waiting_for[robot_id] = None
            
            print(f"Coordinator: {robot_id} granted access to {location}")
            
            response = {
                'robot_id': robot_id,
                'location': location,
                'access_granted': True,
                'queue_position': 0
            }
        
        self.mqtt_client.publish(
            f"{config.MQTT_TOPIC_COORDINATOR}/{robot_id}",
            json.dumps(response)
        )
    
    def handle_access_release(self, data):
        """Handle pelepasan akses dari kasir/rak"""
        robot_id = data['robot_id']
        location = tuple(data['location'])
        
        print(f"Coordinator: {robot_id} releasing access to {location}")
        
        # Release occupation
        if self.location_occupied.get(location) == robot_id:
            self.location_occupied[location] = None
        
        # Process queue
        if location in self.location_queues and self.location_queues[location]:
            # Grant akses ke robot berikutnya di queue
            next_robot = self.location_queues[location].popleft()
            self.location_occupied[location] = next_robot
            self.robot_waiting_for[next_robot] = None
            
            print(f"Coordinator: Granting access to {next_robot} (next in queue)")
            
            # Notify robot berikutnya
            response = {
                'robot_id': next_robot,
                'location': location,
                'access_granted': True,
                'queue_position': 0
            }
            
            self.mqtt_client.publish(
                f"{config.MQTT_TOPIC_COORDINATOR}/{next_robot}",
                json.dumps(response)
            )
    
    def update_robot_state(self, data):
        robot_id = data['robot_id']
        self.robot_states[robot_id] = {
            'status': data['status'],
            'has_cargo': data.get('has_cargo', False),
            'queue_size': data.get('queue_size', 0),
            'is_returning_home': data.get('is_returning_home', False),
            'timestamp': time.time()
        }
        
        if not self.all_robots_moved:
            all_moved = all(
                state.get('status') != 'IDLE' or state.get('queue_size', 0) > 0
                for state in self.robot_states.values()
            )
            if len(self.robot_states) >= config.NUM_ROBOTS and all_moved:
                self.all_robots_moved = True
                print("\n" + "="*60)
                print("Coordinator: All robots have initial tasks - CNP mode active")
                print("="*60 + "\n")
    
    def update_robot_position(self, data):
        robot_id = data['robot_id']
        self.robot_positions[robot_id] = tuple(data['position'])
    
    def handle_task_announcement(self, task_data):
        task_id = task_data['task_id']
        
        if task_id in self.assigned_tasks or task_id in self.announced_tasks:
            return
        
        self.announced_tasks.add(task_id)
        
        print(f"\nCoordinator: New task announced - {task_id}")
        
        if not self.all_robots_moved and self.initial_task_counter < config.NUM_ROBOTS:
            robot_index = self.initial_task_counter
            robot_id = f"ROBOT_{robot_index + 1:02d}"
            
            self.initial_task_counter += 1
            self.assigned_tasks.add(task_id)
            
            print(f"Coordinator: Initial assignment #{self.initial_task_counter} - {task_id} -> {robot_id}")
            
            award = {
                'task_id': task_id,
                'task_data': task_data,
                'winner': robot_id,
                'reason': 'initial_assignment'
            }
            
            self.mqtt_client.publish(
                f"{config.MQTT_TOPIC_TASK_AWARD}/{robot_id}",
                json.dumps(award)
            )
            return
        
        self.pending_tasks[task_id] = task_data
        self.task_bids[task_id] = []
        
        print(f"Coordinator: Requesting bids for {task_id}")
        
        bid_request = {
            'task_id': task_id,
            'task_data': task_data,
            'deadline': time.time() + config.CNP_BID_TIMEOUT
        }
        
        self.mqtt_client.publish(
            config.MQTT_TOPIC_TASK_ANNOUNCE,
            json.dumps(bid_request)
        )
        
        if task_id in self.active_timers:
            self.active_timers[task_id].cancel()
        
        timer = threading.Timer(config.CNP_BID_TIMEOUT, self.evaluate_bids, args=[task_id])
        timer.start()
        self.active_timers[task_id] = timer
    
    def handle_task_bid(self, bid_data):
        task_id = bid_data['task_id']
        
        if task_id not in self.pending_tasks or task_id in self.assigned_tasks:
            return
        
        robot_id = bid_data['robot_id']
        bid_value = bid_data['bid_value']
        
        print(f"Coordinator: Bid from {robot_id} for {task_id} = {bid_value:.2f}")
        self.task_bids[task_id].append(bid_data)
    
    def evaluate_bids(self, task_id: str):
        if task_id not in self.pending_tasks or task_id in self.assigned_tasks:
            return
        
        bids = self.task_bids[task_id]
        
        if not bids:
            print(f"Coordinator: No bids for {task_id}")
            self.announced_tasks.discard(task_id)
            del self.pending_tasks[task_id]
            del self.task_bids[task_id]
            if task_id in self.active_timers:
                del self.active_timers[task_id]
            return
        
        winner_bid = max(bids, key=lambda b: b['bid_value'])
        winner_robot = winner_bid['robot_id']
        
        print(f"\nCoordinator: Task {task_id} awarded to {winner_robot}")
        print(f"  Winning bid: {winner_bid['bid_value']:.2f} (from {len(bids)} bids)")
        
        self.task_assignments[task_id] = winner_robot
        self.assigned_tasks.add(task_id)
        
        award = {
            'task_id': task_id,
            'task_data': self.pending_tasks[task_id],
            'winner': winner_robot,
            'bid_value': winner_bid['bid_value']
        }
        
        self.mqtt_client.publish(
            f"{config.MQTT_TOPIC_TASK_AWARD}/{winner_robot}",
            json.dumps(award)
        )
        
        del self.pending_tasks[task_id]
        del self.task_bids[task_id]
        if task_id in self.active_timers:
            del self.active_timers[task_id]
    
    def handle_reservation_request(self, data):
        robot_id = data['robot_id']
        location = tuple(data['location'])
        action = data['action']
        duration = data.get('duration', config.LOADING_TIME)
        
        current_time = time.time()
        is_available = True
        waiting_time = 0
        
        for reservation in self.location_reservations[location]:
            if reservation['end_time'] > current_time:
                is_available = False
                waiting_time = max(waiting_time, reservation['end_time'] - current_time)
        
        if is_available:
            reservation = {
                'robot_id': robot_id,
                'start_time': current_time,
                'end_time': current_time + duration,
                'action': action
            }
            self.location_reservations[location].append(reservation)
            
            response = {'robot_id': robot_id, 'location': location, 'approved': True, 'waiting_time': 0}
        else:
            response = {'robot_id': robot_id, 'location': location, 'approved': False, 'waiting_time': waiting_time}
        
        self.mqtt_client.publish(f"{config.MQTT_TOPIC_COORDINATOR}/{robot_id}", json.dumps(response))
    
    def handle_release_reservation(self, data):
        robot_id = data['robot_id']
        location = tuple(data['location'])
        
        if location in self.location_reservations:
            self.location_reservations[location] = [
                r for r in self.location_reservations[location] if r['robot_id'] != robot_id
            ]