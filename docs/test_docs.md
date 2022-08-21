<!-- Generated with Stardoc: http://skydoc.bazel.build -->

 Implements functionality for defining ROS tests using rostest.


<a id="ros_test"></a>

## ros_test

<pre>
ros_test(<a href="#ros_test-name">name</a>, <a href="#ros_test-nodes">nodes</a>, <a href="#ros_test-launch_file">launch_file</a>, <a href="#ros_test-launch_args">launch_args</a>, <a href="#ros_test-kwargs">kwargs</a>)
</pre>

 Defines a ROS test.

**PARAMETERS**


| Name  | Description | Default Value |
| :------------- | :------------- | :------------- |
| <a id="ros_test-name"></a>name |  A unique target name.   |  none |
| <a id="ros_test-nodes"></a>nodes |  A list of ROS nodes used by the test.   |  none |
| <a id="ros_test-launch_file"></a>launch_file |  A rostest-compatible launch file.   |  none |
| <a id="ros_test-launch_args"></a>launch_args |  A list of rostest arguments used by the test.   |  <code>None</code> |
| <a id="ros_test-kwargs"></a>kwargs |  https://bazel.build/reference/be/common-definitions#common-attributes-tests   |  none |


