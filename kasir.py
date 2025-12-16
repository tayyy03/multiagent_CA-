import json
import random
import time
import paho.mqtt.client as mqtt
import config

class KasirTerminal:
    def __init__(self, kasir_id: str, kasir_type: str):
        self.kasir_id = kasir_id
        self.kasir_type = kasir_type
        
        self.mqtt_client = mqtt.Client(client_id=kasir_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        try:
            self.mqtt_client.connect(config.MQTT_BROKER, config.MQTT_PORT, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            print(f"MQTT connection failed: {e}")
    
    def on_connect(self, client, userdata, flags, rc):
        print(f"{self.kasir_id} connected to MQTT broker")
        client.subscribe(config.MQTT_TOPIC_TASK_COMPLETE)
    
    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            
            if msg.topic == config.MQTT_TOPIC_TASK_COMPLETE:
                print(f"\n{self.kasir_id} - Task {data['task_id']} completed by {data['robot_id']}")
        
        except Exception as e:
            pass
    
    def send_task(self, task_type: str = None):
        """
        Send random task via coordinator (CNP)
        task_type: 'STORE', 'RETRIEVE', atau None (random)
        """
        task_id = f"TASK_{self.kasir_id}_{int(time.time()*1000)}"
        
        if task_type is None:
            task_type = random.choice(['STORE', 'RETRIEVE'])
        
        pickup_idx = random.randint(0, 4)
        delivery_idx = random.randint(0, 4)
        racks = list(config.RACKS.keys())
        store_rack = random.choice(racks)
        retrieve_rack = random.choice(racks)
        
        task = {
            "task_id": task_id,
            "kasir_id": self.kasir_id,
            "task_type": task_type,
            "pickup_idx": pickup_idx,
            "delivery_idx": delivery_idx,
            "store_rack": store_rack,
            "retrieve_rack": retrieve_rack,
            "timestamp": time.time()
        }
        
        # Announce task to coordinator
        self.mqtt_client.publish(config.MQTT_TOPIC_TASK_ANNOUNCE, json.dumps(task))
        
        if task_type == 'STORE':
            print(f"\n{self.kasir_id} announced {task_type} task {task_id}:")
            print(f"  P{pickup_idx+1} -> Rack {store_rack}")
        else:
            print(f"\n{self.kasir_id} announced {task_type} task {task_id}:")
            print(f"  Rack {retrieve_rack} -> D{delivery_idx+1}")
        
        return task_id
