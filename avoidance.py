import pygame
import math
import sys

# --- Configuration ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
CUBE_SIZE = 40
MOVEMENT_SPEED = 3.0  # Pixels per frame
FPS = 60

# --- Colors ---
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (255, 255, 0)
# --- Start and End Points ---
# The cube will start at this position.
START_POS = [50, 50]

# The cube will move towards this position.
END_POS = (SCREEN_WIDTH - 50, SCREEN_HEIGHT - 50)


def setup_pygame():
    """Initializes Pygame and returns the screen object and clock."""
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Pygame Cube Mover (Start to End)")
    clock = pygame.time.Clock()
    return screen, clock

def calculate_movement_vector(current_pos, target_pos):
    """Calculates the normalized direction vector and the distance to the target."""
    # Calculate the difference in coordinates (the vector)
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]

    # Calculate the distance (magnitude of the vector)
    distance = math.sqrt(dx**2 + dy**2)

    if distance > 0:
        # Normalize the vector (make it a unit vector)
        vx = dx / distance
        vy = dy / distance
        return (vx, vy, distance)
    else:
        # If distance is zero, return zero vector
        return (0, 0, 0)

def dodge_obstacle():
    print("test")


def main():
    """The main game loop."""
    screen, clock = setup_pygame()

    # The cube's current position (using a list for easy modification)
    current_x, current_y = START_POS
    current_pos = [current_x, current_y]

    # Flag to check if the cube has reached the destination
    is_at_end = False

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # Allow quitting via ESC key
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        
        if not is_at_end:
            # 1. Calculate direction and distance
            vx, vy, distance = calculate_movement_vector(current_pos, END_POS)

            # 2. Check if we are close enough to stop (prevent overshooting)
            if distance < MOVEMENT_SPEED:
                current_pos[0] = END_POS[0]
                current_pos[1] = END_POS[1]
                is_at_end = True
                print("Destination Reached!")
            else:
                # 3. Move the cube using the normalized vector and speed
                current_pos[0] += vx * MOVEMENT_SPEED
                current_pos[1] += vy * MOVEMENT_SPEED

        
        screen.fill(BLACK) # Clear the screen

        # Draw the target destination as a small circle
        pygame.draw.circle(screen, RED, END_POS, 5)
        pygame.draw.circle(screen, GREEN, START_POS, 5)

        # Draw the moving cube
        cube_rect = pygame.Rect(
            current_pos[0] - CUBE_SIZE // 2,
            current_pos[1] - CUBE_SIZE // 2,
            CUBE_SIZE,
            CUBE_SIZE
        )
        pygame.draw.rect(screen, BLUE, cube_rect, 0, 8) # Draw a rounded blue cube

        # Update the full display surface
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()