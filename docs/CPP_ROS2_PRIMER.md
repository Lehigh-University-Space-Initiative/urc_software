# C++ and ROS 2 primer for this codebase

A reference for the language features and ROS 2 patterns that show up throughout `urc_software`, written for
someone who has done some programming but is new to C++ or ROS 2. Each section points at a real file in the repo.

You don't need to read this top to bottom. Skim the headings, then come back when a comment in the code says
`Syntax:` and you want more detail.

---

## How the comments in the code are organized

Every first-party source file follows the same comment conventions, so you always know where to look:

| Where | What it tells you |
|-------|-------------------|
| **File header** (top of each file) | What the node/class is, which computer runs it, which launch file starts it, the topics it subscribes and publishes, and how it connects to the rest of the system |
| **Function header** (`Parameters (inputs):` / `Return value:` / `Steps:`) | What the function promises, what goes in and out, and the order it does things in |
| **`// Syntax: ...`** | Explains a C++/Python/ROS language feature, the first time it appears in that file; skip these once you know the feature |
| **`// System: ...`** | Where a topic or parameter connects to another node or computer |
| Ordinary inline comment | Why a line exists, or what a non-obvious value or formula means |

Comments are written one complete thought per line (no sentence wraps across lines), so `clang-format` is configured
with `ReflowComments: false` to leave them alone. See [CONTRIBUTING.md](../CONTRIBUTING.md#comments) for the full rules.

---

## C++ essentials

### Headers and source files

C++ code is split into **headers** (`.h` / `.hpp`: declarations, "this function exists and takes these arguments")
and **source files** (`.cpp`: definitions, the actual code). A `.cpp` file pulls in a header with `#include`.

- `#pragma once` at the top of a header stops it from being included twice in one compile
- `#include "Foo.h"` (quotes) searches this project first; `#include <vector>` (angle brackets) searches system/library paths
- Example: [`src/shared_code/MotorManager.h`](../src/shared_code/MotorManager.h) declares the class,
  [`MotorManager.cpp`](../src/shared_code/MotorManager.cpp) implements it

### `extern` and globals shared between files

`extern rclcpp::Logger dl_logger;` (in [`Logger.h`](../src/shared_code/Logger.h)) says "this variable exists somewhere";
the one real definition lives in `MotorManager.cpp`. That's how several files share one global without each creating a copy.

### Classes, inheritance, and `virtual`

```cpp
class DriveTrainMotorManager : public MotorManager {   // DriveTrainMotorManager IS a MotorManager
    void setupMotors() override;                         // replaces the base class's version
};
```

- `virtual` on a base-class function lets subclasses replace it; a call through a base pointer runs the subclass version
- `= 0` makes a function **pure virtual**: the base class can't be created on its own (it's *abstract*)
- `override` asks the compiler to check you're really replacing a base function (it catches signature typos)
- `public` / `protected` / `private` control who can use a member: everyone / subclasses / only the class itself
- `class SparkMax : CANDriver` (no `public`) inherits **privately** (see [`CANDriver.h`](../src/shared_code/CANDriver.h))
- Examples: `MotorManager` → `DriveTrainMotorManager` / `ArmMotorManager`; `Panel` → every GUI panel

### Constructors and initializer lists

```cpp
Panel::Panel(const std::string& name, const rclcpp::Node::SharedPtr& node)
    : name(name), node_(node)      // members are set here, before the { } body runs
{
}
```

`: CANDriver(canBUS, canID)` in [`CANDriver.cpp`](../src/shared_code/CANDriver.cpp) uses the same syntax to run the
base class's constructor first.

### Pointers, references, and smart pointers

| Syntax | Meaning |
|--------|---------|
| `int& x` | A **reference**: another name for an existing variable (changing `x` changes the original) |
| `int* p` | A raw **pointer**: holds an address; `*p` reads the value, `p->field` reaches a member |
| `std::shared_ptr<T>` | A reference-counted pointer; the object is deleted when the last copy goes away |
| `std::unique_ptr<T>` | A single-owner pointer; the object is deleted when the pointer is destroyed |
| `std::make_shared<T>(args)` / `std::make_unique<T>(args)` | Create a `T` on the heap and wrap it in that smart pointer |

ROS 2 uses `shared_ptr` everywhere: `rclcpp::Node::SharedPtr` is just `std::shared_ptr<rclcpp::Node>`, and every
callback receives its message as `MsgType::SharedPtr msg`, so you read fields with `msg->field`.

### `auto`, range-for, and lambdas

```cpp
auto x = last_joy0_msg_.axes[1];           // compiler works out the type
for (auto& motor : motors_) { ... }         // loop over every element by reference
auto f = [this](const Msg::SharedPtr msg) { this->last = *msg; };   // a lambda (inline function)
```

The `[this]` part is the **capture list**: which outside variables the lambda can use. Capturing `this` lets it reach
the object's members. Lambdas are used as ROS callbacks in the GUI panels
(e.g. [`TelemetryPanel.cpp`](../src/base_station_urc/src/gui/panels/TelemetryPanel.cpp)).

### `std::bind` and `std::placeholders::_1`

```cpp
create_subscription<Joy>("joy0", 10, std::bind(&JoyMapper::joy0Callback, this, std::placeholders::_1));
```

`std::bind` makes a callable that runs `this->joy0Callback(msg)`; `_1` stands for the message the subscription passes
in. It does the same job as a lambda, written the older way. See [`joyMapper.cpp`](../src/base_station_urc/src/joyMapper/joyMapper.cpp).

### `static` (three different meanings)

- **Static member** (`static bool setupCAN(int);` in a class): belongs to the class, shared by all objects
- **Static local** (`static uint64_t loopItr = 0;` in a function): keeps its value between calls
- **Static free function** (`static void reportBusUnavailable(...)` in a `.cpp`): only visible inside that file

### Threads and locks

- `std::thread(fn).detach()` runs `fn` in the background and lets it finish on its own
  (used for pings in [`ComStatusPanel.cpp`](../src/base_station_urc/src/gui/panels/ComStatusPanel.cpp))
- `std::mutex` + `std::lock_guard` stop two threads from touching the same data at once
- `libguarded::plain_guarded<T>` (vendored `cs_libguarded`) bundles a value with its mutex:
  `auto lock = x.lock(); *lock = 5;` locks, writes, and unlocks automatically when `lock` goes out of scope
- `std::atomic<int>` is an integer that's safe to change from several threads without a mutex

### Preprocessor

- `#define MAX_DRIVE_POWER 1.0f` is a text substitution done before compiling (see [`Limits.h`](../src/shared_code/Limits.h))
- `#if defined(__APPLE__)` / `#else` / `#endif` compile only one branch, chosen by platform (see [`Lifecycle.cpp`](../src/base_station_urc/src/gui/Lifecycle.cpp))

---

## ROS 2 essentials

### Nodes, topics, publishers, subscribers

A **node** is one program in the ROS 2 graph. Nodes talk by publishing **messages** on named **topics**; any node can
subscribe to any topic. Nothing connects nodes directly: matching topic names is the whole connection, which is why
every file header in this repo lists the topics it uses.

```cpp
rclcpp::init(argc, argv);                                            // start ROS
auto node = rclcpp::Node::make_shared("DriveTrainManager");          // create a node with a name
auto pub = node->create_publisher<RoverComputerDriveCMD>("roverDriveCommands", 10);
auto sub = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, callback);
pub->publish(msg);
```

The `10` is the **queue depth**: how many unprocessed messages to keep. Full worked example:
[`src/main_computer_urc/src/DriveTrainManager/main.cpp`](../src/main_computer_urc/src/DriveTrainManager/main.cpp).

Topic names without a leading `/` are relative to the node's namespace. Every node here runs in the root namespace,
so `"cmd_vel"` and `"/cmd_vel"` mean the same topic.

### Spinning: how callbacks actually run

Callbacks don't run on their own; something has to **spin** the node:

| Call | Behavior | Used by |
|------|----------|---------|
| `rclcpp::spin(node)` | Runs callbacks forever (until Ctrl+C) | `JoyMapper`, `VideoStreamer` |
| `rclcpp::spin_some(node)` | Runs any ready callbacks, then returns | Nodes with their own main loop (`MotorCtr_node`, the GUI, `WaypointFollower`) |
| `rclcpp::Rate r(10); r.sleep();` | Sleeps so the loop runs at about 10 Hz | Paired with `spin_some` in a `while (rclcpp::ok())` loop |

### Parameters

Parameters are named settings on a node that can be changed at launch time or while running:

```cpp
node->declare_parameter<double>("target_lat", 0.0);              // must declare before reading
double lat = node->get_parameter("target_lat").as_double();
```

```bash
ros2 launch navigation_urc navigation_sim_launch.py target_lat:=38.4070   # set at launch
ros2 param set /WaypointFollower target_lat 38.4070                       # change while running
```

The GUI changes parameters on *other* nodes with `rclcpp::AsyncParametersClient`
(e.g. JoyMapper's `swap_joysticks` from the Telemetry panel).

### Custom messages

The rover's own message types live in [`src/cross_pkg_messages_urc/msg`](../src/cross_pkg_messages_urc/msg). Building
that package generates code from each `.msg` file:

```
msg/RoverComputerDriveCMD.msg  ->  #include "cross_pkg_messages/msg/rover_computer_drive_cmd.hpp"
                                   cross_pkg_messages::msg::RoverComputerDriveCMD     (C++)
                                   from cross_pkg_messages.msg import RoverComputerDriveCMD   (Python)
```

That's why `cross_pkg_messages` is always built first.

### Launch files

A launch file (Python, in each package's `launch/`) starts several nodes together and sets their names, parameters,
and topic remappings. ROS calls its `generate_launch_description()` function. Run one with:

```bash
ros2 launch <package> <file>.py arg:=value
```

[`run_nodes.sh`](../run_nodes.sh) just picks which launch file to run from the Docker run mode.

### ROS 2 command-line tools (great for debugging)

Run these in a second shell inside a container on the same machine (`docker run --rm -it --net=host urc_software manual`):

```bash
ros2 node list                       # which nodes are running
ros2 topic list                      # which topics exist
ros2 topic echo /cmd_vel             # print every message on a topic
ros2 topic info /cmd_vel -v          # who publishes and subscribes to it
ros2 topic hz /video_stream          # how fast it's publishing
ros2 param list /WaypointFollower    # a node's parameters
ros2 interface show cross_pkg_messages/msg/GPSData   # a message's fields
```

### Building with colcon and CMake

`colcon build` builds every ROS package in `src/`. Each package's `CMakeLists.txt` says what to compile:

- `find_package(X REQUIRED)` locates a dependency (it must also be listed in `package.xml`)
- `add_executable(Name src/file.cpp)` compiles a program; `ament_target_dependencies(Name rclcpp ...)` links ROS libraries
- `install(TARGETS Name DESTINATION lib/${PROJECT_NAME})` puts it where `ros2 run` / launch files find it
- `file(GLOB ...)` collects source files by pattern, but only when CMake reconfigures; after adding or removing a
  `.cpp`, do a clean build (`rm -rf build install`)

---

## Python in this repo

Launch files, [`fake_gps_node.py`](../src/navigation_urc/scripts/fake_gps_node.py), and
[`urc_deploy.py`](../softwareUpdate/urc_deploy.py) are Python. ROS 2's Python API (`rclpy`) mirrors the C++ one:

```python
class FakeGPS(Node):                                   # a node is a class that inherits from rclpy's Node
    def __init__(self):
        super().__init__("FakeGPS")                     # node name
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(0.1, self.tick)               # call tick() every 0.1 s
```

`rclpy.spin(node)` plays the same role as `rclcpp::spin`.
