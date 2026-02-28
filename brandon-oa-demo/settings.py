
class Settings:
	_instance = None

	def __new__(cls):
		if cls._instance is None:
			cls._instance = super(Settings, cls).__new__(cls)
			# Constants
			cls._instance.SCREEN_WIDTH = 800
			cls._instance.SCREEN_HEIGHT = 600
			cls._instance.FPS = 60
			# Colors
			cls._instance.WHITE = (255, 255, 255)
			cls._instance.BLACK = (0, 0, 0)
			cls._instance.RED = (255, 0, 0)
			cls._instance.BLUE = (0, 0, 255)
			cls._instance.GREEN = (0, 255, 0)
		return cls._instance
