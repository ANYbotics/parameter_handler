# Use the Parameter Handler in Your Program

## Access the Global Handler

The parameter handler is stored in a global shared pointer:

```cpp
#include "parameter_handler/parameter_handler.hpp"

parameter_handler::handler->...
```

## Select the Handler Type

Available implementations:

### `ParameterHandlerNone`

```cpp
#include "parameter_handler/parameter_handler.hpp"
parameter_handler::setParameterHandlerNone();
```

### `ParameterHandlerStd`

```cpp
#include "parameter_handler_std/parameter_handler_std.hpp"
parameter_handler_std::setParameterHandlerStd();
```

### `ParameterHandlerRos`

```cpp
#include "parameter_handler_ros/parameter_handler_ros.hpp"
parameter_handler_ros::setParameterHandlerRos(&nh);
```

The ROS implementation adds service-based access and was historically used with
an RQT GUI.

## Define and Register Parameters

Parameters use the templated `parameter_handler::Parameter<T>` type:

```cpp
parameter_handler::Parameter<T> myParam_;
```

Example initialization patterns:

```cpp
// Initializer list
MyConstructor()
    : myDoubleParam_("myDouble", 33.33, 11.11, 66.66) {}

// Assignment
myDoubleParam_ =
    parameter_handler::Parameter<double>("myDouble", 33.33, 11.11, 66.66);

// Setters
myDoubleParam_.setName("myDouble");
myDoubleParam_.setValue(33.33);
myDoubleParam_.setDefaultValue(33.33);
myDoubleParam_.setMinValue(11.11);
myDoubleParam_.setMaxValue(66.66);
```

Register a parameter with:

```cpp
parameter_handler::handler->addParam(myDoubleParam_);
```

You can also register an entire struct of parameters in one call:

```cpp
struct HandledParameters {
  ph::Parameter<double> param1_{
      "param1", 0.0, std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::max()};
  ph::Parameter<int> param2_{"param2", 0, -10, 10};
  ph::Parameter<bool> param3_{"param3", false, false, true};
};

HandledParameters parameters;
bool success{ph::registerParameterStruct(parameters)};
```

## Cleanup

Shut the handler down in the destructor of the owning application:

```cpp
parameter_handler::handler->cleanup();
```

## Supported Types

The handler supports:

- integral scalar types such as `bool`, `char`, `short`, `int`, and their
  unsigned variants
- floating-point types such as `float` and `double`
- Eigen integer matrices and vectors such as `Eigen::Matrix3i` and
  `Eigen::VectorXi`
- Eigen floating-point matrices and vectors such as `Eigen::Matrix3d`,
  `Eigen::Vector3f`, and `Eigen::VectorXd`
