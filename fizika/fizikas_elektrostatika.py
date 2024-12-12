import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import os



# Define the simulation grid and parameters
grid_size = 200  # Size of the grid
plate_voltage = 1000  # Voltage on the plates
iterations = 5000  # Number of relaxation iterations
dielectric_constant = 5  # Relative permittivity of the dielectric ring

def initialize_grid(grid_size, plate_voltage):
    """Initialize the simulation grid."""
    grid = np.zeros((grid_size, grid_size))
    grid_set = np.zeros((grid_size, grid_size))
    # Set the plates at the top and bottom
    grid[10, 50:150] = plate_voltage  # Top plate
    grid[170:180, 95:105] = -plate_voltage  # Bottom plate
    
    grid_set = np.where(grid != 0, 1, 0)

    return [grid, grid_set]

def add_conductor_ring(grid, set_potentials):
    """Add a conductor ring to the grid."""
    center = [120, 100]
    radius = 15
    thickness = 2

    for i in range(grid_size):
        for j in range(grid_size):
            dist = np.sqrt((i - center[0]) ** 2 + (j - center[1]) ** 2)
            if radius - thickness <= dist <= radius + thickness:
                grid[i, j] = 0  # Conductor potential remains 0 (grounded)
                set_potentials[i, j] = 1  # Mark as set potential

    return [grid, set_potentials]

def add_dielectric_ring(permittivity, dielectric_constant):
    """Add a dielectric ring to the permittivity grid."""
    permittivity_grid = np.ones_like(permittivity)
    center = np.array([55, 100])
    radius = 15
    thickness = 5

    for i in range(permittivity.shape[0]):
        for j in range(permittivity.shape[1]):
            dist = np.sqrt((i - center[0]) ** 2 + (j - center[1]) ** 2)
            if radius - thickness <= dist <= radius + thickness:
                permittivity_grid[i, j] = dielectric_constant

    return permittivity_grid



def relax_potential(grid, set_potentials, permittivity_grid, iterations):
  for _ in tqdm(range(iterations), desc="Relaxation Progress"):
    new_grid = grid.copy()
    
    # Calculate epsilon values
    epsilon_x = (permittivity_grid[1:, :] + permittivity_grid[:-1, :]) / 2
    epsilon_y = (permittivity_grid[:, 1:] + permittivity_grid[:, :-1]) / 2
    
    # Update grid values where potentials are not set
    for i in range(1, grid.shape[0] - 1):
        for j in range(1, grid.shape[1] - 1):
            if not set_potentials[i, j]:
                new_grid[i, j] = (
                    epsilon_x[i, j] * grid[i + 1, j] +
                    epsilon_x[i - 1, j] * grid[i - 1, j] +
                    epsilon_y[i, j] * grid[i, j + 1] +
                    epsilon_y[i, j - 1] * grid[i, j - 1]
                ) / (epsilon_x[i, j] + epsilon_x[i - 1, j] + epsilon_y[i, j] + epsilon_y[i, j - 1])
    
    grid = new_grid
  return grid
# Initialize the grid
potential_grid, set_potentials = initialize_grid(grid_size, plate_voltage)
permittivity_grid = add_dielectric_ring(np.ones((grid_size, grid_size)), dielectric_constant)
potential_grid, set_potentials = add_conductor_ring(potential_grid, set_potentials)


# Check if the potential data file exists
if os.path.exists('potential.npy'):
    # Load the field data
    final_potential = np.load('potential.npy')
else:
    # Perform relaxation
    final_potential = relax_potential(potential_grid, set_potentials, permittivity_grid, iterations)
    # Save the field data
    np.save('potential.npy', final_potential)


# Calculate electric field (negative gradient of potential)
Ex, Ey = np.gradient(-final_potential)


# Visualize permittivity distribution
plt.figure(figsize=(8, 6))
plt.imshow(permittivity_grid, cmap='seismic', origin='lower')
plt.colorbar(label='Permittivity')
plt.title('Permittivity Distribution')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.show()

# Visualize the results
plt.figure(figsize=(8, 6))
plt.imshow(final_potential, cmap='seismic', origin='lower')
plt.colorbar(label='Potential (V)')
plt.title('Electrostatic Potential Distribution')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.show()

# Visualize electric field lines and equipotential lines
plt.figure(figsize=(8, 6))
plt.contour(final_potential, levels=50, cmap='seismic')
# Overlay dielectric, conductor, and plate outlines
plt.contour(permittivity_grid, levels=[1], colors='blue', linewidths=1, linestyles='dashed', label='Dielectric')
plt.contour(set_potentials, levels=[1], colors='red', linewidths=1, linestyles='solid', label='Conductor')
plt.contour(potential_grid, levels=[plate_voltage], colors='green', linewidths=1, linestyles='dotted', label='Plate')

plt.title('Equipotential Lines')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.show()

# Normalize the electric field vectors
magnitude = np.sqrt(Ex**2 + Ey**2)
Ex_normalized = Ex / magnitude
Ey_normalized = Ey / magnitude

# Visualize the electric field vectors (set to equal magnitude, change color to show magnitude, show only each 5th vector)
plt.figure(figsize=(8, 6))
plt.quiver(np.arange(0, grid_size, 5), np.arange(0, grid_size, 5), Ex_normalized[::5, ::5], Ey_normalized[::5, ::5], magnitude[::5, ::5], scale=2)
plt.title('Electric Field Vectors')
plt.colorbar(label='Magnitude')
plt.show()


