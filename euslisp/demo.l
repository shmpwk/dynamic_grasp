#!/usr/bin/env roseus
;; Author: Shumpei Wakabayashi <wakabayashi@jsk.imi.i.u-tokyo.ac.jp>

(require :softhand-interface "package://softhand_ros/euslisp/softhand-interface.l")
(require "package://eus_vive/euslisp/lib/baxter-interface.l")
(softhand-init)
(eus_vive::baxter-init)
;;(ros::roseus "soft_grasp")
(send *ri* :angle-vector (send *baxter* :reset-pose))

(defun demo ()
  (send *ri* :finger1-rotate (* 3.14 -2.8))
  ;;(send *ri* :finger1-rotate (* 3.14 2.8))
)
(demo)
