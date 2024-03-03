<!-- Generated with Stardoc: http://skydoc.bazel.build -->

Implements code generation functionality for dynamic reconfiguration.

<a id="ros_dynamic_reconfigure_library"></a>

## ros_dynamic_reconfigure_library

<pre>
ros_dynamic_reconfigure_library(<a href="#ros_dynamic_reconfigure_library-name">name</a>, <a href="#ros_dynamic_reconfigure_library-src">src</a>)
</pre>

Defines a rule for storing a dynamic_reconfigure configuration.

**ATTRIBUTES**

| Name                                                  | Description                    | Type                                                               | Mandatory | Default |
| :---------------------------------------------------- | :----------------------------- | :----------------------------------------------------------------- | :-------- | :------ |
| <a id="ros_dynamic_reconfigure_library-name"></a>name | A unique name for this target. | <a href="https://bazel.build/docs/build-ref.html#name">Name</a>    | required  |         |
| <a id="ros_dynamic_reconfigure_library-src"></a>src   | A configuration file (.cfg).   | <a href="https://bazel.build/docs/build-ref.html#labels">Label</a> | required  |         |

<a id="cc_ros_dynamic_reconfigure_library"></a>

## cc_ros_dynamic_reconfigure_library

<pre>
cc_ros_dynamic_reconfigure_library(<a href="#cc_ros_dynamic_reconfigure_library-name">name</a>, <a href="#cc_ros_dynamic_reconfigure_library-dep">dep</a>, <a href="#cc_ros_dynamic_reconfigure_library-kwargs">kwargs</a>)
</pre>

Defines a C++ dynamic reconfiguration library.

**PARAMETERS**

| Name                                                         | Description                                                                    | Default Value |
| :----------------------------------------------------------- | :----------------------------------------------------------------------------- | :------------ |
| <a id="cc_ros_dynamic_reconfigure_library-name"></a>name     | A unique target name.                                                          | none          |
| <a id="cc_ros_dynamic_reconfigure_library-dep"></a>dep       | A configuration file -- a <code>ros_dynamic_reconfigure_library</code> target. | none          |
| <a id="cc_ros_dynamic_reconfigure_library-kwargs"></a>kwargs | https://bazel.build/reference/be/common-definitions#common-attributes          | none          |
