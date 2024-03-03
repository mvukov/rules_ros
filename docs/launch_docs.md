<!-- Generated with Stardoc: http://skydoc.bazel.build -->

Implements functionality for launching ROS deployments using roslaunch.

<a id="ros_launch"></a>

## ros_launch

<pre>
ros_launch(<a href="#ros_launch-name">name</a>, <a href="#ros_launch-nodes">nodes</a>, <a href="#ros_launch-launch_files">launch_files</a>, <a href="#ros_launch-kwargs">kwargs</a>)
</pre>

Defines a ROS deployment.

**PARAMETERS**

| Name                                             | Description                                                                    | Default Value |
| :----------------------------------------------- | :----------------------------------------------------------------------------- | :------------ |
| <a id="ros_launch-name"></a>name                 | A unique target name.                                                          | none          |
| <a id="ros_launch-nodes"></a>nodes               | A list of ROS nodes for the deployment.                                        | none          |
| <a id="ros_launch-launch_files"></a>launch_files | A list of roslaunch-compatible launch files.                                   | none          |
| <a id="ros_launch-kwargs"></a>kwargs             | https://bazel.build/reference/be/common-definitions#common-attributes-binaries | none          |
