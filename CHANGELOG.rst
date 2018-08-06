^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ipcamera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2018-08-06)
------------------

0.1.0 (2018-08-06)
------------------
* docs: Add initial documentation
* feat: Add example calibration file
  - Use it as the camera_info_url for example.launch
* feat: Properly manage Camera Info
  - Use camera_info_manager library to parse camera calibration info from
  a yaml file and publish it through /camera/camera_info topic
* style: Apply clang format
* feat: Add example launch file
* refactor+cleanup: Fix LINT issues
  - Update dependencies
  - Add commands to install the node and launch dir
  - Update maintainers, license and description
* Create CODE_OF_CONDUCT.md
* updated copyrights
* Initial commit
* Contributors: Alireza, alireza, alireza hosseini
