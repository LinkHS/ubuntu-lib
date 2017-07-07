Getting Started with alpha-det-prediction    {#alpha-det-prediction}
=======

# Intro

Alpha-det-prediction library and example code;

# Build

For libalpha-det-predictio.a:

* There are to forms of channelfeatures, S->Y->X->C or S->C->Y->X, which
controlled by two exclusive macros HOBOT_SYXC & HOBOT_SCYX
* HOBOT_SYXC is already defined by default, which can be override by
compiler args
* Use macro SHOW_TIME to debug time use
* NOTE: There are arm specified codes which CANNOT be used for x86 in SYXC,
which auto detected by compiler pre-defined macros

For detect executable binary:

* ./example/path-config.cmake: change the path of opencv and alphadet to your own;

### Dependency

### Environment
* ./build.properties: define the platform & ndk_path...
* ./build.properties.local: personal settings overrides build.properties

# Usage

Two outputs:

* libalpha-det-predictio.a
can be used for other projects
* example/detect
to test functions, use pic list or camera as input

AlphaDet Prediction Module
==============================

This prediction module is for easy integration of AlphaDet into you own projects. It consists of the following parts:

* ./include: head files defining interfaces of AlphaDet;
* ./lib: libraries compiled in different environments, including linux and windows (TODO);
* ./example/models: trained models for testing;
* ./example/detect.cpp: main function to demonstrate how to invoke AlphaDet prediction module;
* ./example/settings.ini.example: an example of necessary settings used by detect.cpp;

Issues when running the program:
* Error happens at line 73 of ./src/mcms.cpp: check the input image size, make sure max_img_w and max_img_h in settings.ini are no smaller than that.

Contact me if you cannot get it work!

Chang Huang @ Horizon Robotics, 2016

Re-edited by Dalong Du @ 20160920

Re-edited by Bo Li @ 20161115


