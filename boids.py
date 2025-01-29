import math
from random import randint, uniform

import pygame as pg
import pygame_gui

# Color constants
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Window Parameters
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Parameters
NUM_BOIDS = 15
BOID_SIZE = 10
SPEED = 3
MAX_FORCE = 0.3
BOID_FRICTION = 0.75

WANDER_RADIUS = 30

SEPARATION = 2
SEPARATION_RADIUS = 40

ALIGNMENT = 1
ALIGNMENT_RADIUS = 50

COHESION = 1
COHESION_RADIUS = 80

PREDATOR_RADIUS = 100
FOOD_POS = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)


class Simulation:

    def __init__(self):
        pg.init()
        self.running = False
        self.clock = pg.time.Clock()
        self.screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.screen_rect = self.screen.get_rect()
        self.fps = 60

        # Set title of window
        pg.display.set_caption("Boids")

        # Load the icon image and set it as the window icon
        icon = pg.image.load('boids.png')
        pg.display.set_icon(icon)

        # Create boids
        self.boids = []
        for i in range(NUM_BOIDS):
            position = (randint(0, SCREEN_WIDTH), randint(0, SCREEN_HEIGHT))
            while any(boid.pos == position for boid in self.boids):
                position = (randint(0, SCREEN_WIDTH), randint(0, SCREEN_HEIGHT))

            self.boids.append(Boid(self, position))
        
        self.predators = [Predator(self, (randint(0, SCREEN_WIDTH), randint(0, SCREEN_HEIGHT)))]

        self.manager = pygame_gui.UIManager((SCREEN_WIDTH, SCREEN_HEIGHT), 'theme.json')

        self.separation_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pg.Rect((50, 10), (100, 25)),  # position for the first slider
            start_value=SEPARATION,
            value_range=(0, 5),
            manager=self.manager
        )
        self.alignment_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pg.Rect((200, 10), (100, 25)),  # position for the second slider
            start_value=ALIGNMENT,
            value_range=(0, 5),
            manager=self.manager
        )
        self.cohesion_slider = pygame_gui.elements.UIHorizontalSlider(
            relative_rect=pg.Rect((350, 10), (100, 25)),  # position for the third slider
            start_value=COHESION,
            value_range=(0, 5),
            manager=self.manager
        )

        # Create UILabels for displaying the values
        self.separation_label = pygame_gui.elements.UILabel(
            relative_rect=pg.Rect((50, 40), (100, 20)),  # position below the first slider
            text=f"Separation: {SEPARATION}",
            manager=self.manager
        )
        self.alignment_label = pygame_gui.elements.UILabel(
            relative_rect=pg.Rect((200, 40), (100, 20)),  # position below the second slider
            text=f"Alignment: {ALIGNMENT}",
            manager=self.manager
        )
        self.cohesion_label = pygame_gui.elements.UILabel(
            relative_rect=pg.Rect((350, 40), (100, 20)),  # position below the third slider
            text=f"Cohesion: {COHESION}",
            manager=self.manager
        )

        

    def events(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False
            elif event.type == pg.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click adds a boid
                    self.boids.append(Boid(self, event.pos))
                    print(f"Added a boid at {event.pos}. Total boids: {len(self.boids)}")
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_p:  # Press 'P' to add a predator
                    position = (randint(0, SCREEN_WIDTH), randint(0, SCREEN_HEIGHT))
                    self.predators.append(Predator(self, position))
                    print(f"Added a predator at {position}. Total predators: {len(self.predators)}")
                elif event.key == pg.K_r:  # Press 'R' to remove the nearest boid
                    mouse_pos = pg.mouse.get_pos()
                    initial_count = len(self.boids)
                    self.boids = [boid for boid in self.boids if boid.pos.distance_to(mouse_pos) > BOID_SIZE * 2]
                    removed_count = initial_count - len(self.boids)
                    if removed_count > 0:
                        print(f"Removed {removed_count} boid(s). Total boids: {len(self.boids)}")


    def draw(self):
        # Empty the last screen
        self.screen.fill(BLACK)
        pg.draw.circle(self.screen, BLUE, FOOD_POS, 10)  # Draw food source

        # Draw all boids
        for boid in self.boids:
            boid.draw(self.screen)
        for predator in self.predators:
            predator.draw(self.screen)

        # Update the screen
        pg.display.update()

    def update(self):
        """
        Method for going one step in the simulation
        """
        mouse_pos = pg.mouse.get_pos()  # Get the current mouse position

        for boid in self.boids:
            boid.update(mouse_pos)

    def run(self):
        """
        Runs the simulation
        """
        self.running = True
        while self.running:
            self.clock.tick(self.fps)
            self.events()
            self.update()
            self.draw()

            time_delta = self.clock.tick(self.fps)/1000.0

            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.running = False

                self.manager.process_events(event)

            self.manager.update(time_delta)
            
            global SEPARATION, ALIGNMENT, COHESION  
            SEPARATION = self.separation_slider.get_current_value()
            ALIGNMENT = self.alignment_slider.get_current_value()
            COHESION = self.cohesion_slider.get_current_value()

            self.separation_label.set_text(f"Separation: {SEPARATION:.2f}")  # update text with current value
            self.alignment_label.set_text(f"Alignment: {ALIGNMENT:.2f}")    # update text with current value
            self.cohesion_label.set_text(f"Cohesion: {COHESION:.2f}")

            # Removed the line that was causing the error
            self.manager.draw_ui(self.screen)

            pg.display.update()

class PhysicsObjet:

    def __init__(self, simulation, position):
        self.simulation = simulation
        self.acc = pg.math.Vector2(0, 0)
        self.vel = pg.math.Vector2(0, 0)
        self.pos = pg.math.Vector2(position)

        self.speed = 1

        self.friction = 0.9

    def update(self):
        self.vel += self.acc
        self.pos += self.vel * self.speed

        # Reset acceleration
        self.acc *= 0


        # Simplistic surface friction
        self.vel *= self.friction
        self.update_energy()

        # wrap around the edges of the screen
        if self.pos.x > self.simulation.screen_rect.w:
            self.pos.x -= self.simulation.screen_rect.w
        elif self.pos.x < 0:
            self.pos.x += self.simulation.screen_rect.w

        if self.pos.y > self.simulation.screen_rect.h:
            self.pos.y -= self.simulation.screen_rect.h
        elif self.pos.y < 0:
            self.pos.y += self.simulation.screen_rect.h


class Boid(PhysicsObjet):

    def __init__(self, simulation, position):
        super().__init__(simulation, position)
        self.speed = SPEED  # Max speed
        self.vel = pg.math.Vector2(randint(-2, 2), randint(-2, 2))  # Random initial velocity

        self.max_force = MAX_FORCE  # force cap, limits the size of the different forces
        self.friction = BOID_FRICTION  # Friction coefficient for the simplistic physics

        # Parameters for wandering behaviour
        self.target = pg.math.Vector2(0, 0)
        self.future_loc = pg.math.Vector2(0, 0)
        self.theta = uniform(-math.pi, math.pi)
        
        self.energy = 100  # Energy for boid


    def update(self, mouse_pos):
        """
        Updates the acceleration of the boid by adding together the different forces that acts on it
        """
        self.acc += self.wander()  # Wandering force
        self.acc += self.separation() * SEPARATION  # separation force scaled with a control parameter
        self.acc += self.alignment() * ALIGNMENT  # alignment force scaled with a control parameter
        self.acc += self.cohesion() * COHESION  # cohesion force scaled with a control parameter
        self.acc += self.avoid_predators()
        self.acc += self.mouse_force(mouse_pos)  # Mouse attraction force

        # move by calling super
        super().update()
    
    def mouse_force(self, mouse_pos):
        """
        Calculate the force to move the boid toward the mouse cursor.
        """
        force = pg.math.Vector2(mouse_pos) - self.pos
        if force.length() > 0:
            force = force.normalize() * self.max_force
        return force

    def avoid_predators(self):
        force = pg.math.Vector2(0, 0)
        for predator in self.simulation.predators:
            distance = self.pos.distance_to(predator.pos)
            if distance < PREDATOR_RADIUS:
                force += (self.pos - predator.pos) / distance  # Move away from predator
        if force.length() > 0:
            force = force.normalize() * self.max_force
        return force

    def update_energy(self):
        self.energy -= 0.1
        if self.pos.distance_to(FOOD_POS) < 20:
            self.energy = min(self.energy + 1, 100)
        if self.energy <= 0:
            self.speed = SPEED / 2
        else:
            self.speed = SPEED

    def separation(self):
        """
        Calculate the separation force vector
        Separation: steer to avoid crowding local flockmates
        :return force vector
        """
        force_vector = pg.math.Vector2(0, 0)
        boids_in_view = self.boids_in_radius(SEPARATION_RADIUS)
    
        # Early return if there are no boids in radius
        if len(boids_in_view) == 0:
            return force_vector
        
        # TODO: Implement this
        for other_boid in boids_in_view:
            distance = self.pos.distance_to(other_boid.pos)

            # Make sure the distance is not 0 to avoid division by 0
            if distance == 0:
                continue

            # Calculate the force vector, the closer the boid is the larger the force
            x_diff = self.pos.x - other_boid.pos.x
            y_diff = self.pos.y - other_boid.pos.y
            force_vector += pg.math.Vector2(x_diff, y_diff) * (SEPARATION_RADIUS / distance)
        
        force_vector = self.cap_force(force_vector, boids_in_view)
        return force_vector

    def alignment(self):
        """
        Calculate the alignment force vector
        Alignment: steer towards the average heading of local flockmates
        :return force vector
        """
        force_vector = pg.math.Vector2(0, 0)
        boids_in_view = self.boids_in_radius(ALIGNMENT_RADIUS)
        
        # Early return if there are no boids in radius
        if len(boids_in_view) == 0:
            return force_vector
        
        # Find the direction of the flock by adding together the velocity vectors of the boids in view
        for other_boid in boids_in_view:
            force_vector += other_boid.vel
            
        if force_vector.length() == 0:
            return force_vector
        
        force_vector = self.cap_force(force_vector, boids_in_view)
        return force_vector

    def cohesion(self):
        """
        Calculate the cohesion force vector
        Cohesion: steer to move toward the average position of local flockmates
        """
        force_vector = pg.math.Vector2(0, 0)
        boids_in_view = self.boids_in_radius(COHESION_RADIUS)
        
        # Early return if there are no boids in radius
        if len(boids_in_view) == 0:
            return force_vector
        
        # Calculate the average position of the boids in view
        other_boid: Boid
        for other_boid in boids_in_view:
            # Make the boids move towards the average position of the boids in view
            dx = other_boid.pos.x - self.pos.x
            dy = other_boid.pos.y - self.pos.y
            force_vector += pg.math.Vector2(dx, dy)

        force_vector = self.cap_force(force_vector, boids_in_view)
        return force_vector

    
    def boids_in_radius(self, radius: float) -> list:
        """
        Find all boids in a given radius
        """
        boids: list = []
        for other_boid in self.simulation.boids:
            if other_boid == self:
                continue

            if self.pos.distance_to(other_boid.pos) < radius:
                boids.append(other_boid)
        return boids

    def cap_force(self, force_vector: pg.math.Vector2, boids_in_view: list) -> pg.math.Vector2:
        """
        Takes a list of boids in view and returns a force vector that is capped by the max force
        """
        force_vector /= len(boids_in_view)
        # Make sure the force vector is not 0
        if force_vector.length() <= 0:
            return force_vector
        
        force_vector = force_vector.normalize() * self.speed - self.vel

        if force_vector.length() > self.max_force:
            force_vector.scale_to_length(self.max_force)
        return force_vector

    def move_towards_target(self, target):
        """
        Calculate force vector for moving the boid to the target
        """
        # vector to the target
        desired = target - self.pos

        distance = desired.length()
        desired = desired.normalize()

        # Radius
        radius = 100

        if distance < radius:
            # if the distance is less than the radius,
            m = remap(distance, 0, radius, 0, self.speed)

            # scale the desired vector up to continue movement in that direction
            desired *= m
        else:
            desired *= self.speed

        force_vector = desired - self.vel
        limit(force_vector, self.max_force)
        return force_vector

    def wander(self):
        """
        Calcualte a random target to move towards to get natural random flight
        """
        if self.vel.length_squared() != 0:
            # Calculate where you will be in the future
            self.future_loc = self.vel.normalize() * 80

            # Calculate a random angle addition
            self.theta += uniform(-math.pi, math.pi) / 10

            # set the target to your position + your future position + a distance in the direction of the random angle
            self.target = self.pos + self.future_loc + pg.math.Vector2(WANDER_RADIUS * math.cos(self.theta),
                                                                       WANDER_RADIUS * math.sin(self.theta))
        return self.move_towards_target(self.target)

    def draw(self, screen):
        """Draw boid to screen"""

        # Calculate the angle to the velocity vector to get the forward direction
        angle = math.atan2(self.vel.y, self.vel.x)
        other_points_angle = 0.75 * math.pi  # angle +- value to get the other two points in the triangle

        # Get the points of the triangle
        x0 = self.pos.x + BOID_SIZE * math.cos(angle)
        y0 = self.pos.y + BOID_SIZE * math.sin(angle)

        x1 = self.pos.x + BOID_SIZE * math.cos(angle + other_points_angle)
        y1 = self.pos.y + BOID_SIZE * math.sin(angle + other_points_angle)

        x2 = self.pos.x + BOID_SIZE * math.cos(angle - other_points_angle)
        y2 = self.pos.y + BOID_SIZE * math.sin(angle - other_points_angle)

        # Draw
        pg.draw.polygon(screen, WHITE, [(x1, y1), (x2, y2), (x0, y0)])

class Predator:
    def __init__(self, simulation, position):
        self.simulation = simulation
        self.pos = pg.math.Vector2(position)
        self.vel = pg.math.Vector2(uniform(-2, 2), uniform(-2, 2))
        self.acc = pg.math.Vector2(0, 0)
        self.speed = SPEED * 1.5

    def update(self):
        # Find the closest boid
        closest_boid = min(self.simulation.boids, key=lambda b: self.pos.distance_to(b.pos), default=None)
        if closest_boid:
            # Move toward the closest boid
            desired = (closest_boid.pos - self.pos).normalize() * self.speed
            self.acc = desired - self.vel

        # Update velocity and position
        self.vel += self.acc
        if self.vel.length() > self.speed:
            self.vel.scale_to_length(self.speed)
        self.pos += self.vel

        # Reset acceleration
        self.acc *= 0

        # Wrap around screen edges
        self.wrap_around_screen()

    def wrap_around_screen(self):
        if self.pos.x > SCREEN_WIDTH:
            self.pos.x = 0
        elif self.pos.x < 0:
            self.pos.x = SCREEN_WIDTH
        if self.pos.y > SCREEN_HEIGHT:
            self.pos.y = 0
        elif self.pos.y < 0:
            self.pos.y = SCREEN_HEIGHT

    def draw(self, screen):
        pg.draw.circle(screen, RED, (int(self.pos.x), int(self.pos.y)), BOID_SIZE + 5)

# Helper functions
def remap(n, start1, stop1, start2, stop2):
    """Remap a value in one range to a different range"""
    new_value = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2
    if start2 < stop2:
        return constrain(new_value, start2, stop2)
    else:
        return constrain(new_value, stop2, start2)


def constrain(n, low, high):
    """Constrain a value to a range"""
    return max(min(n, high), low)


def limit(vector, length):
    """Cap a value"""
    if vector.length_squared() <= length * length:
        return
    else:
        vector.scale_to_length(length)


if __name__ == '__main__':
    sim = Simulation()
    sim.run()
