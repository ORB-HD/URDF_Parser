URDF Parser
===========

This is a small URDF (Universal Robot Descriptor File) Parser library. It is intended to be
used to load the most important information needed to display a robot. It was created to
serve as a simple static library to be used in rbdl-toolkit and the rbdl library. One of the main desing goals was to keep it simple and provide a consistent error handling that may be
used to find errors in the URDF files that are loaded.

Insipiration was drawn from the urdfdom library and just as urdfdom it uses tinyxml as
the xml implementation.

Feel free to use it in your own projects.
