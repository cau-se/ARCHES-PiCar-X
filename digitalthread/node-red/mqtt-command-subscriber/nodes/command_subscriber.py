#!/usr/bin/env python3

from picarx_msgs.msg import Drive
import rospy
import argparse
from typing import Dict, Union
import paho.mqtt.client as mqtt
import json
from arches_mqtt_bridge.client import MQTTClientFactory


class CommandSubscriberOptions(object):

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The name of the digital shadow skill")
        parser.add_argument(
            "uid", type=str, help="The unique identifier of the digital shadow skill. Leave blank if you want an automated generation.")
        parser.add_argument(
            "mqtt_topic", type=str, help="The MQTT topic the command is published to.")

        self.args = parser.parse_args(argv)

    def get_args(self):
        """
        Returns the parsed command line arguments.

        :return: Parsed command line arguments.
        :rtype: dict
        """
        return vars(self.args)


class CommandSubscriberMQTT(object):

    def __init__(self, name: str, uid: str = None, mqtt_topic: str = "/picarx/drive/command") -> None:
        self.name = name
        self.uid = uid
        self.__client = None
        self.mqtt_topic = mqtt_topic
        self.command_publisher = None

    @property
    def client(self) -> mqtt.Client:
        return self.__client

    @client.setter
    def client(self, config: Dict) -> mqtt.Client:
        if config is not None:
            self.__client = MQTTClientFactory.create_client(
                rospy.get_param("~config", None))
            self.__init_client()
        else:
            self.__client = None

    @property
    def mqtt_topic(self) -> str:
        return self.__mqtt_topic

    @mqtt_topic.setter
    def mqtt_topic(self, topic: Union[str, None]) -> Union[str, None]:
        self.__mqtt_topic = topic

    @property
    def command_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the command publisher.

        Returns:
            Union[None, rospy.Publisher]: The command publisher.
        """
        return self.__command_publisher

    @command_publisher.setter
    def command_publisher(self, publisher: rospy.Publisher) -> Union[None, rospy.Publisher]:
        """
        Publish drive commands to the ROS network.

        Args:
            publisher (rospy.Publisher): The command publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__command_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__command_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    def __init_client(self) -> None:
        self.client.on_connect = self.__on_connect
        self.client.on_disconnect = self.__on_disconnect
        self.client.on_message = self._on_message

    def __on_connect(self, client, userdata, flags, rc) -> None:
        rospy.loginfo("Connected to MQTT master with result code "+str(rc))
        self.client.subscribe(self.mqtt_topic)

    def __on_disconnect(self, client, userdata, rc=0):
        rospy.logdebug("Disconnected result code "+str(rc))
        self.client.loop_stop()

    def _on_message(self, client, userdata, msg: str) -> None:
        rospy.loginfo(msg)
        mqtt_message = json.loads(msg.payload)
        ros_message = Drive()
        rospy.loginfo(mqtt_message)
        ros_message.speed = mqtt_message['speed']
        ros_message.angle = mqtt_message['angle']
        self.command_publisher.publish(ros_message)

    def start(self) -> None:
        rospy.init_node("-")
        self.command_publisher = rospy.Publisher(
            rospy.get_param('~command_topic'), Drive, queue_size=5)
        rospy.loginfo("hi")
        self.client = rospy.get_param("~config")
        rospy.on_shutdown(self.client.disconnect)
        self.client.loop_start()
        rospy.spin()


if __name__ == "__main__":
    start_arguments = CommandSubscriberOptions(rospy.myargv()[1:])
    driving_skill = CommandSubscriberMQTT(
        start_arguments.args.name, start_arguments.args.uid, start_arguments.args.mqtt_topic)
    driving_skill.start()
