#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

void relax_potential(std::vector<std::vector<double>> &grid, const std::vector<std::vector<bool>> &set_potentials, const std::vector<std::vector<double>> &permittivity_grid, int iterations)
{
  int rows = grid.size();
  int cols = grid[0].size();

  for (int iter = 0; iter < iterations; ++iter)
  {
    std::vector<std::vector<double>> new_grid = grid;

    // Calculate epsilon values
    std::vector<std::vector<double>> epsilon_x(rows - 1, std::vector<double>(cols));
    std::vector<std::vector<double>> epsilon_y(rows, std::vector<double>(cols - 1));

    for (int i = 0; i < rows - 1; ++i)
    {
      for (int j = 0; j < cols; ++j)
      {
        epsilon_x[i][j] = (permittivity_grid[i + 1][j] + permittivity_grid[i][j]) / 2.0;
      }
    }

    for (int i = 0; i < rows; ++i)
    {
      for (int j = 0; j < cols - 1; ++j)
      {
        epsilon_y[i][j] = (permittivity_grid[i][j + 1] + permittivity_grid[i][j]) / 2.0;
      }
    }

    // Update grid values where potentials are not set
    for (int i = 1; i < rows - 1; ++i)
    {
      for (int j = 1; j < cols - 1; ++j)
      {
        if (!set_potentials[i][j])
        {
          new_grid[i][j] = (epsilon_x[i][j] * grid[i + 1][j] +
                            epsilon_x[i - 1][j] * grid[i - 1][j] +
                            epsilon_y[i][j] * grid[i][j + 1] +
                            epsilon_y[i][j - 1] * grid[i][j - 1]) /
                           (epsilon_x[i][j] + epsilon_x[i - 1][j] + epsilon_y[i][j] + epsilon_y[i][j - 1]);
        }
      }
    }

    grid = new_grid;
  }
}