import pygame
import numpy as np
import numpy.typing as npt
from typing import Generator

def animate_frames(height: int, width: int, frames: Generator[npt.NDArray[np.uint8], None, None]):
    pygame.init()

    # Create a Pygame surface from a NumPy array representing the pixel buffer
    pixels_rgb = np.zeros((width, height, 3), dtype=np.uint8)
    surface = pygame.surfarray.make_surface(pixels_rgb)

    # Create a Pygame display
    display = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Live signal')

    # Main loop
    running = True
    while running:
        # Convert grayscale pixels to rgb pixels
        pixels_grayscale = next(frames)
        pixels_grayscale.shape = (height, width)
        pixels_grayscale = pixels_grayscale.T
        pixels_grayscale.shape = (width, height)
        pixels_rgb = np.stack((pixels_grayscale,)*3, axis=-1)

        # Blit new pixels
        pygame.surfarray.blit_array(surface, pixels_rgb)
        display.blit(surface, (0, 0))

        # Update the display
        pygame.display.flip()

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

def test_frames():
    i = 0
    while True:
        yield np.array([i % 256]*300*300, dtype=np.uint8)
        i += 1

if __name__ == "__main__":
    animate_frames(300, 300, test_frames())
