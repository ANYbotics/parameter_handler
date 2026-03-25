# Extend the Parameter Handler With Additional Types

## Add a New Type

To add a new parameter type:

1. Add traits for the new type to `ParameterValueTraits.hpp`.
2. Extend `ParameterHandlerRos.hpp` with services that can set and get the new
   type.
3. Add the corresponding GUI support and visualization widgets for that type.

Even if min and max values are part of the parameter definition, a trait does
not have to enforce them. For example, strings could use `min = max = ""`
without interpreting those limits.

## Add a New Eigen Matrix Type

The parameter handler already supports many Eigen matrix types. If you also
want to expose a matrix type through the ROS interface or the RQT GUI, add it
to `type_macros.hpp`.

Only matrix types whose scalar types are already supported can be added there.
