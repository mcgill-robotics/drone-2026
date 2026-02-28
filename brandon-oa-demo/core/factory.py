class RobotFactory:
    @staticmethod
    def create_robot(x, y, obstacles, course, config=None, navigation_strategy=None):
        from robot import Robot
        if config is None:
            config = ConfigFactory.create_config()
        return Robot(x, y, obstacles, course, config=config, navigation_strategy=navigation_strategy)

class ObstacleFactory:
    @staticmethod
    def create_obstacle(x, y, charge=500.0, diam=20.0):
        from obstacle import Obstacle
        return Obstacle(x, y, charge, diam)

class ConfigFactory:
    @staticmethod
    def create_config():
        from config.robot_config import RobotConfig
        return RobotConfig()
