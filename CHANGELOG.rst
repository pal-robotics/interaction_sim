^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package interaction_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2024-11-29)
------------------
* launch: force colorized output
* launch: tell communication_hub that the chatbot is available under /chatbot/...
* add dependency on image-transport-plugins
  while here, re-order the dependency by alphabetical order
* correct camera name in the provided default camera calibration file
* Contributors: Séverin Lemaignan

0.6.0 (2024-11-27)
------------------
* additional gscam adaptations
  - added a predefined camera configuration file to config folder
  - gscam to use such configuration file
  - updated rqt perspective to use by default the /camera ns
* use gscam instead of usb_Cam
  usb_cam does not offer the sensor QoS, which seems to cause issues
* add missing dependency on hri_emotion_models
* Contributors: Séverin Lemaignan, lorenzoferrini

0.5.1 (2024-10-16)
------------------
* update rqt perspective with correct topic for hri_visualization
* disable expressive_eyes foreground filter to improve compression
* Contributors: Séverin Lemaignan

0.5.0 (2024-10-16)
------------------
* start emotion_recognizer
* remove volume_control_pulseaudio from launch file
* add dep on hri_visualization
* use launch arguments instead of config file, to avoid impacting people's config
* Contributors: Séverin Lemaignan

0.4.0 (2024-10-14)
------------------
* add attention_manager and rqt_human_radar
* Contributors: Séverin Lemaignan

0.3.1 (2024-10-09)
------------------
* add missing upstream dep on pydantic
* start the knowledge base
* auto activate communication_hub
* Contributors: Séverin Lemaignan

0.3.0 (2024-10-08)
------------------
* also launch rqt with pre-set window configuration
* Contributors: Séverin Lemaignan

0.2.1 (2024-10-08)
------------------
* volume_control_pulseaudio is not actually needed (this is a dep of RASA chatbots)
* Contributors: Séverin Lemaignan

0.2.0 (2024-10-08)
------------------
* add missing dependencies
* minor: typo in README
* [doc] basic readme
* initial metapkg for the interaction simulator
* Contributors: Séverin Lemaignan
