
A simple V-REP plug-in for controlling a quadcopter.

The quadcopter model must be tagged with custom data so the plug-in
can find it.  See the included scene and source for details.

The simulated GPS sensor requires the "GeographicLib" library,
which can be installed from:

  http://www.geographiclib.sourceforge.net/

The Makefile assumes that geographiclib is in the default search
path, such as a typical installation to "/usr/local".
