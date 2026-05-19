# Control System Algorithms

[![Linux CI](https://github.com/Control-Edge/cs-algorithms/actions/workflows/Linux%20CI.yml/badge.svg)](https://github.com/Control-Edge/cs-algorithms/actions/workflows/Linux%20CI.yml)
[![macOS CI](https://github.com/Control-Edge/cs-algorithms/actions/workflows/macOS%20CI.yml/badge.svg)](https://github.com/Control-Edge/cs-algorithms/actions/workflows/macOS%20CI.yml)
[![Windows CI](https://github.com/Control-Edge/cs-algorithms/actions/workflows/Windows%20CI.yml/badge.svg)](https://github.com/Control-Edge/cs-algorithms/actions/workflows/Windows%20CI.yml)

Collection of C and C++ algorithms for control system and embedded software development. Includes reusable implementations for signal processing, filtering, numerical methods, control logic, diagnostics, and simulation utilities designed for real-time, resource-constrained, and industrial applications.

## How to use

1. Download latest CppModel libraries from your workspace and place them in the `dependencies/` directory.
2. Configure and build the project using CMake:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```
3. Run the simulation:
   ```bash
   ./simulation_executable
   ```
4. Adjust parameters and setpoints in the simulation code or through the CppModel workspace to test different scenarios.

## Repository Structure

- `algorithms/`: Core algorithm implementations for control systems, signal processing, and numerical methods.
- `simulations/`: Example simulations demonstrating the use of algorithms in control system scenarios.

The repository is hosted on github.com and pipelines are set to use any test enabled in simulations as a CI test. So any failure in the simulations will cause the pipeline to fail, ensuring that all algorithms are tested and validated.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your improvements or new algorithms. Make sure to include documentation for any new code you add. Use simulations as test and demonstration of your algorithms.

## Community and Support

For questions, discussions, or support, please open an issue on the GitHub repository or join our community channel on Discord (https://discord.gg/p8X7EJdW). We encourage collaboration and are happy to help with any issues you may encounter while using or contributing to the project.


