<!-- Generated with Stardoc: http://skydoc.bazel.build -->

 Defines commonly used macros.


<a id="cc_ros_binary"></a>

## cc_ros_binary

<pre>
cc_ros_binary(<a href="#cc_ros_binary-name">name</a>, <a href="#cc_ros_binary-ros_package_name">ros_package_name</a>, <a href="#cc_ros_binary-kwargs">kwargs</a>)
</pre>

 Defines a ROS cc_binary.

Adds common ROS definitions on top of a cc_binary.


**PARAMETERS**


| Name  | Description | Default Value |
| :------------- | :------------- | :------------- |
| <a id="cc_ros_binary-name"></a>name |  A unique target name.   |  none |
| <a id="cc_ros_binary-ros_package_name"></a>ros_package_name |  If given, defines a ROS package name for the target. Otherwise, name is used as the package name.   |  <code>None</code> |
| <a id="cc_ros_binary-kwargs"></a>kwargs |  https://bazel.build/reference/be/common-definitions#common-attributes-binaries   |  none |


<a id="cc_ros_library"></a>

## cc_ros_library

<pre>
cc_ros_library(<a href="#cc_ros_library-name">name</a>, <a href="#cc_ros_library-ros_package_name">ros_package_name</a>, <a href="#cc_ros_library-kwargs">kwargs</a>)
</pre>

 Defines a ROS cc_library.

Adds common ROS definitions on top of a cc_library.


**PARAMETERS**


| Name  | Description | Default Value |
| :------------- | :------------- | :------------- |
| <a id="cc_ros_library-name"></a>name |  A unique target name.   |  none |
| <a id="cc_ros_library-ros_package_name"></a>ros_package_name |  If given, defines a ROS package name for the target. Otherwise, name is used as the package name.   |  <code>None</code> |
| <a id="cc_ros_library-kwargs"></a>kwargs |  https://bazel.build/reference/be/common-definitions#common-attributes   |  none |


