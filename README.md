# Bazel rules for ROS

This repo aims to build ROS (1) from scratch.

## Prerequisites

You will need to install Bazel, see [here](https://docs.bazel.build/versions/master/install.html).
Besides Bazel, you will need a C++ compiler and a Python 3 interpreter.

And no, you don't have to install any ROS packages via `apt`.

The code is developed and tested on Ubuntu 22.04 with Python 3.10.

## What works?

So far a subset of ros-base packages can be built, including support for

- messages,
- services,
- actions, and
- dynamic reconfiguration.

Please take a look at the [examples](examples) folder to get started.

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
code to filter out unwanted deps (mainly the ones that require compiled Python
extensions in roslaunch). Next, `roslib` has complex dependencies
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
