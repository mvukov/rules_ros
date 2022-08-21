<!-- Generated with Stardoc: http://skydoc.bazel.build -->

 Implements a macro for setting up target-dependent rostopic app.


<a id="ros_topic"></a>

## ros_topic

<pre>
ros_topic(<a href="#ros_topic-name">name</a>, <a href="#ros_topic-deps">deps</a>, <a href="#ros_topic-kwargs">kwargs</a>)
</pre>

 Defines rostopic app for a set of deps.

**PARAMETERS**


| Name  | Description | Default Value |
| :------------- | :------------- | :------------- |
| <a id="ros_topic-name"></a>name |  A unique target name.   |  none |
| <a id="ros_topic-deps"></a>deps |  A list of deps for which all <code>ros_interface_library</code> targets are collected and on which this target can operate on. This would typically be a list of ROS node targets or ROS deployments (<code>ros_launch</code> targets).   |  none |
| <a id="ros_topic-kwargs"></a>kwargs |  https://bazel.build/reference/be/common-definitions#common-attributes-binaries   |  none |


