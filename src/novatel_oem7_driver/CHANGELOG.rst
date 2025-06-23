^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_oem7_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

24.1.0 (2025-03-17)
--------------------
Odometry Bug Fixes

Modifications:
* Odometry is no longer published when the receiver position type = None
* Similar Changes as Release 20.7.0 for Humble


24.0.0 (2025-01-31)
--------------------
Formal support for Jazzy

Modifications:

* Upgraded oem7_receiver_if.hpp to remove Boost dependency
* Upgraded port and net receiver connections to ensure that they are non-blocking
* Removed Boost dependency from the port and net receiver connections. This removes the ability to build with Windows. These files have been left but labelled as sync (to indicate they are blocking), these files are deprecated and will be removed in a future release 
* Updates to address various deprecation warnings by third party libraries


20.6.0 (2025-01-30)
--------------------
Various QoL changes

Modifications:

* Reduced log verbosity of various expected logs to DEBUG
* Updated launch files to include all configurable parameters 
* Added BESTGNSSVEL and RAWIMUSX Oem7 Messages
* Expanded oem7_msgs.yaml and std_oem7_raw_msgs.yaml to include all topics matching Oem7Decoder database
* Minor adjustments to std_init_commands.yaml


20.5.0 (2024-09-26)
--------------------
Misc ROS2 fixes
Added support in the CMakeLists.txt to include building with ninja


20.4.0 (2024-07-25)
--------------------
Implementation of Odometry to publish ellipsoid height for GNSS only and SPAN receivers

Fixes:

* Adjusted Odometry to use undulation from BESTPOS or INSPVAX
* Listed git as a dependency in CmakeLists.txt

20.0.0 (2023-04-18)
--------------------
Formal support for Humble; functionality updated to that of ROS1 v4.2.0

Features:

* BESTGNSSPOS, PPPPOS, TERRASTARINFO, TERRASTARSTATUS Oem7 Messages
     
* imu/data_raw output, source from RAWIMUSX and scaled
    
* HG4930_AN04, HG4930_AN04_400Hz IMUs
  
* Odometry Angular velocities

* Optionally, publish Odometry Transform

* Optionally, use first valid GPSFix as Odometry Pose origin


Fixes:

* Rotate Odometry Twist covariances into local frame



10.5.0 (2021-11-12)
--------------------
Misc ROS2 fixes


10.0.0 (2021-08-31)
--------------------
Initial Support for ROS2/Foxy


2.2.0 (2021-02-03)
------------------
* No feature changes

* More robust support for compillation for 32 vs. 64 bit targets


2.1.0 (2021-01-28)
------------------

* Support for ARM builds: arm32v7, arm64v8 (`#1 <https://github.com/novatel/novatel_oem7_driver/issues/1>`_)

* Initialization command mechanism robustness improvements


2.0.0 (2020-12-18)
------------------
* Support logging 'unknown' OEM7 messages under Oem7Raw topic
   
  
* Populate gps_common/GPSStatus.status with more detailed status info

* Source gps_common/GPSFix, gps_common/NavSatFix position data from the most recent and higher quality
  INSPVAS or BESTPOS/BESTVEL messsage.
  
  Previously, position data was always sourced from BESTPOS/BESTVEL, which is transmitted
  at lower rate than INSPVAS
  (`#13 <https://github.com/novatel/novatel_oem7_driver/issues/13>`_)   
* Support nmea_msgs/Sentence for all OEM7 NMEA0183 messages (`#4 <https://github.com/novatel/novatel_oem7_driver/issues/4>`_)

* Support nav_msgs/Odometry, with position and orientation populated based on BESTPOS/BESTVEL/INSPVAS
  (`#8 <https://github.com/novatel/novatel_oem7_driver/issues/8>`_)



1.1.0 (2020-05-02)
------------------------
* No feature changes

  Build fixes and documentation improvements.

1.0.0 (2020-04-20)
------------------------------
* Initial release


