# A chatter example

Let's begin with starting `roscore`:

```sh
bazel run //:roscore
```

In a separate terminal, let's start a (C++) talker node:

```sh
bazel run //chatter:talker
```

This single command will compile and run the talker node.

In a yet another terminal we can start a listener node:

```sh
bazel run //chatter:listener  # C++ version or
bazel run //chatter:py_listener  # Python version
```

Rosbag recording & playing works as well:

```sh
bazel run //:rosbag_record -- /chatter -o /tmp/foo.bag  # to record a bag or
bazel run //:rosbag_play -- /tmp/foo_<timestamp>.bag  # to play a bag
```

`rostopic`, tied to this example (see `BUILD.bazel` for more info) can be used as

```sh
bazel run //chatter:rostopic -- echo /chatter
```

Not too shabby.

Next, let's start a deployment with the talker and the listener nodes. You can
stop the nodes you started with the above commands. Now execute

```sh
bazel run //chatter:chatter
```

This command will build the necessary nodes and launch them. This is similar
to executing good-ol' `roslaunch`, but, running the chatter `ros_launch` target
using Bazel ensures all necessary dependencies are (re-)built.
