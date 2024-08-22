class AckermannStartOptions(object):
    """Start argument parser to the digital shadow skill.
        You have to set the name and a unique id. If you leave out a unique id,
        an autoincrement value will be used as id.

        Note:
            If you use Docker, we recommand to set the ids.
    """

    def __init__(self, argv):
        """
        Initialize the argument parser with command line arguments.

        Args:
            argv (list): List of command line arguments.
        """
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The name of the digital shadow skill")
        parser.add_argument(
            "uid", type=str, help="The unique identifier of the digital shadow skill. Leave blank if you want and automated generation.")

        self.args = parser.parse_args(argv)


class AckermannDriveSkill(Skill):
    """
    AckermannDriveSkill class for controlling the Ackermann drive.

    Attributes:
        command_subscriber (rospy.Subscriber): Subscriber for receiving commands.
        motor_left_publisher (rospy.Publisher): Publisher for the left motor.
        motor_right_publisher (rospy.Publisher): Publisher for the right motor.
        clutchgeer_publisher (rospy.Publisher): Publisher for the clutch gear.
        status_publisher (rospy.Publisher): Publisher for the drive status.
    """

    def __init__(self, name: str, uid: str = None) -> None:
        """
        Initialize the AckermannDriveSkill.

        Args:
            name (str): Name of the skill.
            uid (str, optional): Unique identifier of the skill. Defaults to None.
        """
        super(AckermannDriveSkill, self).__init__(name, SkillType.CONTROL, uid)
        self.command_subscriber = None
        self.motor_left_publisher = None
        self.motor_right_publisher = None
        self.clutchgeer_publisher = None
        self.status_publisher = None

    @property
    def command_subscriber(self) -> Union[None, rospy.Subscriber]:
        """
        Get the command subscriber.

        Returns:
            Union[None, rospy.Subscriber]: The command subscriber.
        """
        return self.__command_subscriber

    @command_subscriber.setter
    @DigitalThread.control
    def command_subscriber(self, subscriber: rospy.Subscriber) -> Union[None, rospy.Subscriber]:
        """
        Set the command subscriber.

        Args:
            subscriber (rospy.Subscriber): The command subscriber.

        Raises:
            ValueError: If the subscriber is not of type rospy.Subscriber or None.
        """
        if subscriber is None:
            self.__command_subscriber = None
        elif isinstance(subscriber, rospy.Subscriber):
            self.__command_subscriber = subscriber
        else:
            raise ValueError(
                "Subscriber has to be of type rospy.Subscriber or None, but {} was given.".format(type(subscriber)))

    @property
    def status_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the status publisher.

        Returns:
            Union[None, rospy.Publisher]: The status publisher.
        """
        return self.__status_publisher

    @status_publisher.setter
    @DigitalThread.data
    def status_publisher(self, publisher: rospy.Publisher) -> Union[None, rospy.Publisher]:
        """
        Set the status publisher.

        Args:
            publisher (rospy.Publisher): The status publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__status_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__status_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def motor_left_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the left motor publisher.

        Returns:
            Union[None, rospy.Publisher]: The left motor publisher.
        """
        return self.__motor_left_publisher

    @motor_left_publisher.setter
    def motor_left_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        """
        Set the left motor publisher.

        Args:
            publisher (Union[None, rospy.Publisher]): The left motor publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__motor_left_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_left_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def motor_right_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the right motor publisher.

        Returns:
            Union[None, rospy.Publisher]: The right motor publisher.
        """
        return self.__motor_right_publisher

    @motor_right_publisher.setter
    def motor_right_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        """
        Set the right motor publisher.

        Args:
            publisher (Union[None, rospy.Publisher]): The right motor publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__motor_right_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_right_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def clutchgeer_publisher(self) -> Union[None, rospy.Publisher]:
        """
        Get the clutch gear publisher.

        Returns:
            Union[None, rospy.Publisher]: The clutch gear publisher.
        """
        return self.__clutchgeer_publisher

    @clutchgeer_publisher.setter
    def clutchgeer_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        """
        Set the clutch gear publisher.

        Args:
            publisher (Union[None, rospy.Publisher]): The clutch gear publisher.

        Raises:
            ValueError: If the publisher is not of type rospy.Publisher or None.
        """
        if publisher is None:
            self.__clutchgeer_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__clutchgeer_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    def drive(self, ros_msg: Any) -> None:
        """
        Drive the vehicle based on the received ROS message.

        Args:
            ros_msg (Any): The ROS message containing drive commands.
        """
        # There are problems with understeering in the Ackermann approximation in the simulation, thus we limitate
        # the steering angle to [-20, 20] degree, until the understeering is fixed. Max angle of the physical twin
        # is around |35| degree.
        if ros_msg.angle > 20:
            angle = 20
        elif ros_msg.angle < -20:
            angle = -20
        else:
            angle = ros_msg.angle

        self.motor_left_publisher.publish(Int8(ros_msg.speed))
        self.motor_right_publisher.publish(Int8(ros_msg.speed))
        self.clutchgeer_publisher.publish(Int8(angle))

    def send_status(self, motor_left_status: MotorStatus, motor_right_status: MotorStatus, clutchgear_status: ClutchGearStatus) -> None:
        """
        Send the status of the drive components.

        Args:
            motor_left_status (MotorStatus): Status of the left motor.
            motor_right_status (MotorStatus): Status of the right motor.
            clutchgear_status (ClutchGearStatus): Status of the clutch gear.
        """
        timestamp = rospy.get_rostime()
        status_message = DriveStatus(timestamp=timestamp, motor_left=motor_left_status,
                                     motor_right=motor_right_status, clutchgear=clutchgear_status)
        self.status_publisher.publish(status_message)

    def init_status_filter(self, motor_left_filter, motor_right_filter, steering_filter):
        """
        Initialize the status filter for synchronizing messages.

        Args:
            motor_left_filter: Filter for the left motor status.
            motor_right_filter: Filter for the right motor status.
            steering_filter: Filter for the steering status.
        """
        ts = message_filters.ApproximateTimeSynchronizer(
            [motor_left_filter, motor_right_filter, steering_filter], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.send_status)

    def stop(self):
        """
        Stop the skill and log the shutdown.
        """
        rospy.loginfo("SHUTTING DOWN")
