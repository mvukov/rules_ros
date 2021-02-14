# Bazel rules for ROS

This repo aims to build ROS (1) from scratch.

## Prerequisites

The code is developed and tested on Ubuntu 20.04 with Python 3.8.

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).

Besides Bazel, you will need some additional deps:

```sh
sudo apt install libboost-dev-all libbz2-dev liblz4-dev
```

plus a C++ compiler and a Python 3.8 interpreter. At least on Ubuntu 20.04,
you'll need to also do the following:

```sh
sudo ln -s /usr/bin/python3 /usr/bin/python
```

Bazel-generated driver scripts for Python binaries still use `/usr/bin/python`.

And no, you don't have to install any ROS packages via `apt`.

## What works?

So far a subset of ros-core packages can be built.

Here is an example.

Let's begin with starting ROS master:

```sh
bazel run //:rosmaster
```

In a separate terminal, let's start a (C++) talker node:

```sh
bazel run //examples:talker
```

This single command will compile and run the talker node.

In a yet another terminal we can start a listener node:

```sh
bazel run //example:listener  # C++ version or
bazel run //example:py_listener  # Python version
```

Rosbag recording & playing works as well:

```sh
bazel run //:rosbag_record -- /chatter -o /tmp/foo.bag  # to record a bag or
bazel run //:rosbag_play -- /tmp/foo_<timestamp>.bag  # to play a bag
```

`rostopic`, tied to this example (see `examples/BUILD.bazel` for more info) can
be used as

```sh
bazel run //examples:rostopic -- echo /chatter
```

Not too shabby.

## What about `roslaunch`?

Getting `roscpp` `rosmaster`, and `rosbag` up-n-running wasn't a lot of work.
`rospy`and `rostopic` required some hacking of `roslib`, but this was still
not a big effort.

Naturally, for me at least, was to try to get `roslaunch` working. After "a bit"
I figured out that `roslaunch` is so tightly coupled with catkin/ROS package
management that I put those developments on hold. It will probably be the best
to write something similar that can work with Bazel with some custom rules.

## Where is this heading?

I have no solid plans. I did this for fun to learn more about Bazel. Before I
started working on this I saw a yet another effort to use ROS from Bazel in
https://github.com/nicolov/ros-bazel and was not too happy how message
generation was implemented. So I decided to make an alternative implementation
on my own -- I wanted it to look and feel more like exisiting protobuf rules for
Bazel. In addition, I wanted to see how difficult would it be to get rid of
`catkin` :) `catkin`+`CMake` do a great job FWIW. However, Bazel workflow is
simpler, at least from the perspective of roboticists who want to develop
robotics software (included me) and are not much interested in build
procedures.

I think it would be cool if we would have a single e.g. `ros_launch`
Bazel rule that can (build and) run a whole deployment of ROS nodes. That way it
would be fairly easy, I believe, to pack the deployment in an archive or a
Docker image (using convenient rules from [rules_docker](https://github.com/bazelbuild/rules_docker)).
Deployment of binaries to robots would be simplified this way, I believe.

## Additional

Optionally you can install `poetry` for resolving/updating Python deps:

```sh
sudo python3.8 -m pip install poetry
```

Then you can run the script `./generate_python_requirements.sh` to update
Python deps.
