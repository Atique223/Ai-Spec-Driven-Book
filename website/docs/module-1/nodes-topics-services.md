---
sidebar_position: 3
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

The fundamental communication patterns in ROS 2 are built around three core concepts: nodes, topics, and services. Understanding these patterns is crucial for designing effective robotic systems that can coordinate complex behaviors.

## Nodes

A node is a process that performs computation in ROS. Nodes are the fundamental building blocks of a ROS system. In Python, you create nodes using the `rclpy` library:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Node initialization code here
```

### Node Characteristics:
- Each node should have a unique name within the ROS graph
- Nodes can be written in different programming languages
- Nodes communicate with each other through topics, services, and actions
- Nodes can be started and stopped independently

## Topics - Publish-Subscribe Pattern

Topics enable asynchronous, one-way communication using a publish-subscribe pattern. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic.

### Publisher Example:
```python
import rclpy
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
```

### Subscriber Example:
```python
import rclpy
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Topic Communication Features:
- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Loose Coupling**: Publishers don't know who subscribes to their topics
- **Multiple Publishers/Subscribers**: Multiple nodes can publish/subscribe to the same topic
- **Message Types**: Defined using `.msg` files with strict type definitions

## Services - Request-Response Pattern

Services provide synchronous, two-way communication using a request-response pattern. A client sends a request and waits for a response from the service server.

### Service Server Example:
```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Service Client Example:
```python
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions - Goal-Oriented Communication

Actions are used for long-running tasks that may take time to complete and can be preempted. They combine the features of topics and services.

## Communication Patterns in Humanoid Robotics

In humanoid robots, these patterns are used for:

- **Topics**: Sensor data streaming (camera feeds, IMU data, joint states)
- **Services**: Configuration changes, calibration, mode switching
- **Actions**: Complex behaviors like walking, grasping, or navigation

## Knowledge Check

1. When would you use a topic versus a service in a robotic system?
2. What are the advantages and disadvantages of publish-subscribe communication?
3. How might you use these communication patterns in a humanoid robot's walking controller?

## Practical Exercise

Design a communication architecture for a humanoid robot's head movement system. Consider what nodes you would need, what topics/services they would use, and how they would coordinate to enable smooth head movement with visual tracking.