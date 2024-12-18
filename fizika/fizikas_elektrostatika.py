import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import os




# Define the simulation grid and parameters
grid_size = 200  # Size of the grid
plate_voltage = 220  # Voltage on the plates
iterations = 5000  # Number of relaxation iterations
dielectric_constant = 5  # Relative permittivity of the dielectric ring

def initialize_grid(grid_size, plate_voltage):
    grid = np.zeros((grid_size, grid_size))
    grid_set = np.zeros((grid_size, grid_size))
    # Set the plates at the top and bottom
    grid[10, 50:150] = plate_voltage  # Top plate
    grid[170:180, 95:105] = -plate_voltage  # Bottom plate
    
    grid_set = np.where(grid != 0, 1, 0)

    return [grid, grid_set]

def add_conductor_ring(grid, set_potentials):
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
    permittivity_grid = np.ones_like(permittivity)
    center = np.array([55, 100])
    radius = 15
    thickness = 2

    for i in range(permittivity.shape[0]):
        for j in range(permittivity.shape[1]):
            dist = np.sqrt((i - center[0]) ** 2 + (j - center[1]) ** 2)
            if radius - thickness <= dist <= radius + thickness:
                permittivity_grid[i, j] = dielectric_constant

    return permittivity_grid



def relax_potential(grid, set_potentials, permittivity_grid, iterations):
  for _ in tqdm(range(iterations), desc="Relaxation Progress"):
    new_grid = grid.copy()
    
    # Calculate epsilon values. Brought out of the loop for optmization 
    epsilon_x = (permittivity_grid[1:, :] + permittivity_grid[:-1, :]) / 2
    epsilon_y = (permittivity_grid[:, 1:] + permittivity_grid[:, :-1]) / 2
    
    # Update grid values where potentials are not set for more details why this works for laplace equations check the link below
    # https://physics.stackexchange.com/questions/310447/explanation-of-relaxation-method-for-laplaces-equation
    
    for i in range(1, grid.shape[0] - 1):
        for j in range(1, grid.shape[1] - 1):
            if not set_potentials[i, j]:
                new_grid[i, j] = (
                    epsilon_x[i, j] * grid[i + 1, j] +
                    epsilon_x[i - 1, j] * grid[i - 1, j] +
                    epsilon_y[i, j] * grid[i, j + 1] +
                    epsilon_y[i, j - 1] * grid[i, j - 1]
                ) / (epsilon_x[i, j] + epsilon_x[i - 1, j] + epsilon_y[i, j] + epsilon_y[i, j - 1])
    
    
    delta = np.abs(new_grid - grid).max()
    # Update the progress bar description with the current delta
    tqdm.write(f"Delta: {delta:.6e}")
    grid = new_grid

 
    
  return grid
# Initialize the grid
potential_grid, set_potentials = initialize_grid(grid_size, plate_voltage)
permittivity_grid = add_dielectric_ring(np.ones((grid_size, grid_size)), dielectric_constant)
potential_grid, set_potentials = add_conductor_ring(potential_grid, set_potentials)


# Check if the potential data file exists
if os.path.exists('potential(2).npy'):
  # Load the field data
  final_potential = np.load('potential(2).npy')
  
  if input('Do you want to perform relaxation? (y/n)') == 'y':
    final_potential = relax_potential(final_potential, set_potentials, permittivity_grid, int(input('Enter the number of iterations: ')))
    # Save the field data
    np.save('potential(2).npy', final_potential)
else:
  # Perform relaxation
  final_potential = relax_potential(potential_grid, set_potentials, permittivity_grid, iterations)
  # Save the field data
  np.save('potential(2).npy', final_potential)





#---------------------------------VISUALIZATION---------------------------------#





# Calculate electric field (negative gradient of potential)
Ey, Ex = np.gradient(-final_potential)


#Visualize the setup of permittivity and set potentials side by side
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.imshow(permittivity_grid, cmap='viridis', origin='lower')
plt.colorbar(label='Permittivity')
plt.title('Permittivity Distribution')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

plt.subplot(1, 2, 2)
plt.imshow(set_potentials, cmap='plasma', origin='lower')
plt.colorbar(label='Set Potentials')
plt.title('Potential Distribution')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show()


# Visualize the results
plt.figure(figsize=(8, 6))
plt.imshow(final_potential, cmap='seismic', origin='lower')
plt.colorbar(label='Potential (V)')
plt.title('Electrostatic Potential Distribution')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show()

# Visualize electric field lines and equipotential lines
plt.figure(figsize=(8, 6))
plt.contour(final_potential, levels=100, colors='red', linewidths=0.5, linestyles='solid', label='Equipotential Lines')
# Overlay dielectric, conductor, and plate outlines
plt.contour(permittivity_grid, levels=[2], colors='blue', linewidths=1, linestyles='dashed', label='Dielectric')
plt.contour(set_potentials, colors='blue', levels=[0], linewidths=1, linestyles='dashed', label='Conductor')
# show labels
plt.legend()

plt.title('Equipotential Lines')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show()

# Normalize the electric field vectors
magnitude = np.sqrt(Ex**2 + Ey**2)
magnitude[magnitude == 0] = 1  # Avoid division by zero
size = 0.1
Ex_normalized = (Ex / magnitude) * size
Ey_normalized = (Ey / magnitude) * size
magnitude = np.clip(magnitude, 0, 10)
magnitude = np.log1p(magnitude)**0.5  # Logarithmic scaling

# Visualize the electric field vectors (set to equal magnitude, change color to show magnitude, show only each 5th vector)
plt.figure(figsize=(8, 6))
X, Y = np.meshgrid(np.arange(0, grid_size, 5), np.arange(0, grid_size, 5))
plt.quiver(X, Y, Ex_normalized[::5, ::5], Ey_normalized[::5, ::5], magnitude[::5, ::5], scale=5, cmap='viridis')
plt.contour(permittivity_grid, levels=[1], colors='blue', linewidths=1, linestyles='dashed')
plt.contour(set_potentials, colors='blue', levels=[0], linewidths=1, linestyles='dashed', label='Conductor')
plt.title('Electric Field Vectors')
plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show()

# Visualize the electric field lines
plt.figure(figsize=(8, 6))
plt.streamplot(np.arange(0, grid_size), np.arange(0, grid_size), Ex, Ey, color=magnitude, linewidth=1, cmap='viridis', density=3, integration_direction='both')
plt.contour(permittivity_grid, levels=[1], colors='blue', linewidths=1, linestyles='dashed')
plt.contour(set_potentials, colors='blue', levels=[0], linewidths=1, linestyles='dashed', label='Conductor')
plt.title('Electric Field Lines')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show()

# visualize the 2nd derivative of the potential
plt.figure(figsize=(8, 6))
plt.imshow(np.gradient(np.gradient(final_potential)[0])[0], cmap='seismic', origin='lower')
plt.colorbar(label='Second Derivative of Potential')
plt.title('Second Derivative of Potential Distribution')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show()


