This repo details development of a low-cost ice tracking and distributed snow depth measurement system. LiDAR-SWARM integrates single-beam lidar sensors, LoRa radios, and space-based data relay to bridge the [Internet of Things]( https://en.wikipedia.org/wiki/Internet_of_things) to remote Arctic locations. Effectively, we enable deployment of a [LoRaWAN](https://lora-alliance.org/resource_hub/what-is-lorawan/)-like environment where traditional internet connectivity is impossible.  The system integrates 3 main components, however only the first is a requirement:

1.	[Swarm Eval Kit](https://swarm.space/products)
2.	[Benewake LiDAR](http://en.benewake.com)
3.	[Dragino LoRa End Node](https://www.dragino.com/products/lora-lorawan-end-node.html)</li>

The core of the hardware is a Swarm modem which provides connectivity to a constellation of satellites with packet relay up to 192 bytes per message at 1 kbps. Coverage of the constellation is [global]( https://kube.tools.swarm.space/pass-checker), with great temporal coverage and ability to queue messages. For rapid prototyping I’ve built around the Swarm Eval Kit which integrates a satellite modem, solar panel, batteries, and microcontroller. The Eval Kit uses an ESP32-S2 in the [Adafruit Feather]( https://learn.adafruit.com/adafruit-feather/feather-specification) specification as its microcontroller. I’ve modified the Eval Kit to use the (Adafruit LoRa radio FeatherWing)[ https://www.adafruit.com/product/3231] with stacking headers to enable LoRa communication on site.

![System workflow](pics/ lidarswarm_flow.jpg)

Remote sensor nodes are used to collect environmental data, in this case snow depth measurements and air temperature. For the prototype system I’ve used off the shelf Dragino LSN50-V2 nodes but in a more generalized application, anything with a LoRa modem could be used if a single channel transmit can be set. Sensors in this application are single beam lidars which measure distance from the node to the surface. Variance in this distance shows changes in snow depth. Again, in a more generalized application the end-nodes can integrate whatever sensor is convenient for the application. The end nodes transmit the sensor data at a set interval in the LoRAWAN packet specification.

![lidar node](pics/ lidarswarm_sensor.jpg)

Firmware at the Swarm node coordinates LoRAWAN packet reception, JSON-like repackaging, and thereon, transmission to space with the Swarm satellite radio. The packaged data includes timestamped, geolocation (lat/lon), altitude, snow depth, air temperature. We can queue up to 2000 packets and relay them where there is satellite availability. Data posts to the Swarm Hive and thereon is transferred to a preferred dashboarding service via Webhooks. The whole system is fairly low cost (<$1000 build + $5 a month for connectivity) and provides distributed LoRA connectivity at remote locations for the first time.

![swarm node](pics/ lidarswarm_eval.jpg)