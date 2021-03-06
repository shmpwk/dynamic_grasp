#!/usr/bin/env roseus
;; Author: Shumpei Wakabayashi <wakabayashi@jsk.imi.i.u-tokyo.ac.jp>
(require "package://eus_vive/euslisp/lib/baxter-interface.l")
(ros::roseus "util")

(eus_vive::baxter-init)
(setq *lif* (send *ri* :get-gripper-interface :larm))

;;(send *lif* :finger3-rotate (* 3.14 -2.0))
