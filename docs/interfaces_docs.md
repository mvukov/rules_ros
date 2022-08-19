<!-- Generated with Stardoc: http://skydoc.bazel.build -->

 Implements functionality for code generation of ROS interfaces.

Inspired by code in https://github.com/nicolov/ros-bazel repo.


<a id="ros_interface_library"></a>

## ros_interface_library

<pre>
ros_interface_library(<a href="#ros_interface_library-name">name</a>, <a href="#ros_interface_library-deps">deps</a>, <a href="#ros_interface_library-srcs">srcs</a>)
</pre>



**ATTRIBUTES**


| Name  | Description | Type | Mandatory | Default |
| :------------- | :------------- | :------------- | :------------- | :------------- |
| <a id="ros_interface_library-name"></a>name |  A unique name for this target.   | <a href="https://bazel.build/docs/build-ref.html#name">Name</a> | required |  |
| <a id="ros_interface_library-deps"></a>deps |  -   | <a href="https://bazel.build/docs/build-ref.html#labels">List of labels</a> | optional | [] |
| <a id="ros_interface_library-srcs"></a>srcs |  -   | <a href="https://bazel.build/docs/build-ref.html#labels">List of labels</a> | required |  |


<a id="cc_ros_interface_library"></a>

## cc_ros_interface_library

<pre>
cc_ros_interface_library(<a href="#cc_ros_interface_library-name">name</a>, <a href="#cc_ros_interface_library-deps">deps</a>, <a href="#cc_ros_interface_library-kwargs">kwargs</a>)
</pre>

 Defines a C++ ROS interface library.

**PARAMETERS**


| Name  | Description | Default Value |
| :------------- | :------------- | :------------- |
| <a id="cc_ros_interface_library-name"></a>name |  The target name.   |  none |
| <a id="cc_ros_interface_library-deps"></a>deps |  A list of deps (list of ros_interface_library targets).   |  none |
| <a id="cc_ros_interface_library-kwargs"></a>kwargs |  https://bazel.build/reference/be/common-definitions#common-attributes   |  none |


<a id="py_ros_interface_library"></a>

## py_ros_interface_library

<pre>
py_ros_interface_library(<a href="#py_ros_interface_library-name">name</a>, <a href="#py_ros_interface_library-deps">deps</a>, <a href="#py_ros_interface_library-kwargs">kwargs</a>)
</pre>

 Defines a Python ROS interface library.

**PARAMETERS**


| Name  | Description | Default Value |
| :------------- | :------------- | :------------- |
| <a id="py_ros_interface_library-name"></a>name |  The target name:   |  none |
| <a id="py_ros_interface_library-deps"></a>deps |  A list of deps (list of ros_interface_library targets).   |  none |
| <a id="py_ros_interface_library-kwargs"></a>kwargs |  https://bazel.build/reference/be/common-definitions#common-attributes   |  none |


