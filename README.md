# Bazel rules for ROS

This repo aims to build ROS (1) from scratch.

## Prerequisites

The code is developed and tested on Ubuntu 20.04 with Python 3.8.

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).
Besides Bazel, you will need a C++ compiler and a Python 3.8 interpreter. If you
want to run ROS deployments in Docker containers, [install Docker](https://docs.docker.com/engine/install/ubuntu/)
as well.

At least on Ubuntu 20.04, you'll need to also do the following:

```sh
sudo ln -s /usr/bin/python3 /usr/bin/python
```

Bazel-generated driver scripts for Python binaries still use `/usr/bin/python`.

And no, you don't have to install any ROS packages via `apt`.

## What works?

So far a subset of ros-base packages can be built, including support for
- messages,
- services,
- actions, and
- dynamic reconfiguration.

Here is an example.

Let's begin with starting `roscore`:

```sh
bazel run //:roscore
```

In a separate terminal, let's start a (C++) talker node:

```sh
bazel run //examples/chatter:talker
```

This single command will compile and run the talker node.

In a yet another terminal we can start a listener node:

```sh
bazel run //examples/chatter:listener  # C++ version or
bazel run //examples/chatter:py_listener  # Python version
```

Rosbag recording & playing works as well:

```sh
bazel run //:rosbag_record -- /chatter -o /tmp/foo.bag  # to record a bag or
bazel run //:rosbag_play -- /tmp/foo_<timestamp>.bag  # to play a bag
```

`rostopic`, tied to this example (see `examples/chatter/BUILD.bazel` for more
info) can be used as

```sh
bazel run //examples/chatter:rostopic -- echo /chatter
```

Not too shabby.

Next, let's start a deployment with the talker and the listener nodes. You can
stop the nodes you started with the above commands. Now execute

```sh
bazel run //examples/chatter:chatter
```

This command will build the necessary nodes and launch them. This is similar
to executing good-ol' `roslaunch`, but, running the chatter `ros_launch` target
using Bazel ensures all necessary dependencies are (re-)built.

How about executing the chatter deployment within a Docker container?
Just run

```sh
bazel run //examples/chatter:chatter_image
```

FYI, the size of a compressed image made in release mode (with `--config=opt`)
is less than 50MB -- check [here](https://hub.docker.com/r/mvukov/chatter/tags?page=1&ordering=last_updated).
A very simple base image used for the example chatter image can be found in
`docker/base`.

In `//examples/dishwasher` you can find another example that demonstrates
defining and usage of ROS actions (and actionlib).

## Packaging

By running

```sh
bazel build //examples/chatter:chatter_pkg
```

one can create the chatter deployment and package it in an archive. This is
interesting for deployment on devices different than the build machine.

## Cross-compilation

You can cross-compile the chatter deployment for e.g. [NVIDIA's Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) with

```sh
bazel build //examples/chatter:chatter --config=jetson-opt
```

You don't have to install a cross-compiler, Bazel will download one for you.
Neat, right? On it's own, this is not yet super interesting as one has to
deploy the cross-compiled binaries to a target device. The binaries can be
packaged with

```sh
bazel build //examples/chatter:chatter_pkg --config=jetson-opt
```

All you need to deploy to the target device is in `bazel-bin/examples/chatter/chatter_pkg.tar.gz`.

After you transfer the archive to the target, you can do the following at the
target:

```bash
sudo mkdir -p /app  # This is a (configurable) root folder for the deployment.
sudo tar -xvzf chatter_pkg.tar.gz -C /
```

and then you can start the deployment with

```bash
/app/examples/chatter/chatter
```

## Additional

Optionally you can install `pip-tools` for resolving/updating Python deps:

```sh
sudo python3.8 -m pip install pip-tools
```

Then you can run the script `./generate_python_requirements.sh` to update
Python deps.

## Background and design decisions

Within this project I want to learn and practice Bazel as well as to learn more
about how difficult and/or feasible is to use ROS with Bazel. I started the work
for desktop (amd64 architecture) but the real goal is to have cross-compilation
for e.g. arm64 architecture working out of the box.
Moreover, the number of deps that need to be installed on the target machine
should be minimal, e.g. only C++ and Python runtimes.

For C++ this means that the build system needs to cross-compile all ROS and
application deps, and for Python packages this means that all of them should be
native -- i.e. without compiled extensions. For this project, I tried to find
alternatives with Python-only code where possible, otherwise I removed pieces of
code that depend on such deps. The end effect is that the core&base ROS packages
used in this repo use only native Python deps. In other words, Python deps are
handled by Bazel and you don't have to install (almost) any Python packages
on the target platform.

It turned out that handling C++ dependencies is not that difficult. Some of the
packages, mainly from `ros_comm` repo, have been fixed along the way. I believe
that those changes can be eventually merged into the main `ros_comm` repo and
that is why those changes are at the moment in a fork.

On the Python side, situation was more difficult. I had to refactor `roslaunch`
code to filter out non-wanted deps. Next, `roslib` has complex dependencies
and I kept only strictly necessary parts. Changes to `rosservice` are minimal.
(Heavily) modified Python ROS packages are stored in `//third_party` and you can
inspect `git` history to get more info about the changes I made.

Regarding development for embedded platforms, I believe `roscpp` should be just
fine. `rospy` has some deps that have compiled extensions, so, `rospy` should
not be used for platforms other than amd64 at the moment.

Since plugin functionality depends heavily on `roslib` and `rospack`,
which I don't intend to touch any more, I won't work on ROS plugin support.
Going to a bit more detail: `rospack` calls Python from C++ which tremendously
complicates development for embedded platforms.
