---
title: The ARCHES Digital Twin Framework
has_children: false
nav_order: 4
parent: Explore
---

# The ARCHES Digital Twin Framework

# ARCHESPublisher and ARCHESSubscriber extend ROS Publishers and Subscribers

![ARCHES Publisher and Subscriber](../assets/images/adtf-pubsubs.png "UML ARCHES Digital Twin Framework publisher and subscriber") 

# The Role of Skills in the ADTF

## Definition
A **Skill** describes the capability that is added to the system by introducing a microservice. Services and drivers can be part of a Skill, but only the nodes that contain publishers or subscribers that are able to synchronize their messages with the physical twin or digital twin are referred to as **Skill**.

## Implement as Skill
Examples of Skills provided by ADTF include the **DigitalShadow-Collector* and *DigitalShadow-Distributor* services. The collector gathers data for its twin, while the distributor forwards received data to corresponding topics. Leveraging the publish-subscribe architecture, we can seamlessly tap into publishers and subscribers for data exchange.

```python
@Skill('ExampleSkill')
class SensorSkill:
    @property
    def dataPublisher():
        return self.dataPublisher
    
    @Digitalshadow.data
    @dataPublisher.setter
    def dataPublisher(publisher: Publisher):
        self.dataPublisher = publisher
    
    @property
    def commandSubscriber():
        return self.commandSubscriber
    
    @Digitalshadow.control
    @commandSubscriber.setter
    def commandSubscriber(subscriber: Subscriber):
        self.commandSubscriber = subscriber
        
    def __init__() {
        self.dataPublisher = new Publisher('/A/B/C', StandardO2, queue_size = 5)
        self.commandSubscriber = new Subscriber('incoming/command', 
            Command, self.commandCallback)
    }
    
    def commandCallback(rosmsg: Command) {
        // do something here
    }
```

## PiCar-X Example

# A Dynamic Compact Control Language with Apache AVRO
`StandardO2` ROS messages:
```json
{
'Time': {'secs': 1554119012,'nsecs': 513111}, 
'Oxy': 234.87, 
'Sat': 104.7503,
'Temp': 28.78
}
```

```
27 80 D1 42 B8 DE 6A 43 71 3D E6 41 C8 E5 8F CA 0B 94 D1 AB E9 03
```

```
\mathrm{61 72 63 68 65 73 5F 6D 73 67 73 2F 53 74 61 6E 64 61 72 64 4F 32}\\
\mathrm{00} \\
\mathrm{27 80 D1 42 B8 DE 6A 43 71 3D E6 41 C8 E5 8F CA 0B 94 D1 AB E9 03}
```

```
10 00 2780D142B8DE6A43713DE641C8E58FCA0B94D1ABE903
```

`HeaderTwin` ROS message definition:

```json
string skillID
string topicID
```

```json
{
'header': { 'skillId': 'exampleSkill', 'topicID': '/topic' },
'Time': {'secs': 1554119012,'nsecs': 513111}, 
'Oxy': 234.87, 
'Sat': 104.7503,
'Temp': 28.78
}
```