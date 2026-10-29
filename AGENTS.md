# Repository Guidelines

## Project Structure

`elevator-walking` is a ROS 2 colcon workspace. Add ROS packages under `src/<package_name>/`. Each package should include `package.xml` and either `CMakeLists.txt` (ament_cmake) or `setup.py`/`setup.cfg` (ament_python). Keep node code in `src/` or the Python package directory, launch files in `launch/`, parameters in `config/`, and tests in `test/`. Do not commit generated `build/`, `install/`, or `log/` directories.

## Build, Test, and Run

Use the ROS 2 distribution targeted by the packages; Foxy is installed in this environment. In a clean shell, run:

```bash
source /opt/ros/foxy/setup.bash
cd /home/unitree/elevator-walking
colcon build --symlink-install
source install/setup.bash
ros2 run <package_name> <executable>
```

Run package tests with `colcon test --packages-select <package_name>` and review results with `colcon test-result --verbose`. Do not mix ROS distributions in one shell.

## Coding Style and Naming

Use lowercase snake_case for package names, Python modules, functions, and variables; use PascalCase for classes. Follow PEP 8 with four-space Python indentation. For C++, use `.hpp` headers, `.cpp` sources, two-space indentation, and snake_case for functions and variables. Reuse a formatter or linter only after it is added to the package; none is configured yet.

## Testing Guidelines

Place tests in each package's `test/` directory. Name Python tests `test_*.py` and C++ tests `*_test.cpp`. Keep unit tests independent of a physical robot; document the ROS distribution and hardware setup for any integration or robot smoke test.

## Commits and Pull Requests

This workspace is not a Git repository yet, so no commit convention can be inferred. When version control is initialized, use imperative Conventional Commit subjects such as `feat: add relative movement node` or `fix: stop on odometry timeout`. PRs should summarize behavior changes, list build/test commands and results, and state whether physical robot validation was performed.

## Configuration

Keep robot-specific settings and credentials out of version control. Commit safe example configuration files and document required environment variables or network settings.

## Research and Uncertainty

When behavior, APIs, or compatibility are unclear, search the repository, upstream source, and official documentation before deciding or editing. Prefer primary sources and note relevant links and versions in the change description. If uncertainty remains, state assumptions and ask for clarification instead of guessing.
