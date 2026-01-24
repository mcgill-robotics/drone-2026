import pygame

class WallObstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.rect = pygame.Rect(x - width // 2, y - height // 2, width, height)
        self.is_trap = True
        self.color = (0, 0, 139)  # Dark blue
    
    def draw(self, surface):
        pygame.draw.rect(surface, self.color, self.rect)
        pygame.draw.rect(surface, (0, 0, 0), self.rect, 2)  # Black border