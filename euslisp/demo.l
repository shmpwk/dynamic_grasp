#!/usr/bin/env roseus
;; Author: Shumpei Wakabayashi <wakabayashi@jsk.imi.i.u-tokyo.ac.jp>

(defvar *grasp-target-bounding-box-topic* "/bounding_box_marker/selected_box")
(defvar *grasp-status-topic* "/tabletop_object_grasp_status")
(defvar *base-frame-id* "/base")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_rviz_plugins")
(ros::roseus-add-msgs "geometry_msgs")
(require "models/arrow-object.l")
(require :softhand-interface "package://softhand_ros/euslisp/softhand-interface.l")
(require "package://eus_vive/euslisp/lib/baxter-interface.l")

(ros::roseus "soft_grasp")
(ros::rate 10)

(defun baxter-pregrasp-pose ()
  ;;(send *ri* :start-grasp)
  (send *ri* :angle-vector (send *baxter* :reset-pose) 5000 nil 0)
  ;;(send *ri* :finger1-rotate (* 3.14 -2.8))
  ;;(send *ri* :start-grasp :larm (* 3.14 -2.8))
  (setq *co-endeffector-vis* (arrow))
  (setq *co-ik-target* (arrow))
  (setq *pregrasp-point* (arrow))
  (setq *grasp-point* (arrow))
  (setq *box-arrow* (arrow))
  (objects (list *co-endeffector-vis* *co-ik-target* *baxter*))
  )

(defun publish-status (msg-type fmt)
  (let ((color
         (case msg-type
          ('error (instance std_msgs::ColorRGBA :init
                            :r 1.0 :g 0.0 :b 0.0 :a 1.0))
          ('warn (instance std_msgs::ColorRGBA :init
                           :r 1.0 :g 1.0 :b 0.0 :a 1.0))
          (t (instance std_msgs::ColorRGBA :init
                       :r (/ 157.0 255) :g (/ 204.0 255) :b (/ 224.0 255) :a 1.0)))))
    (unless (ros::get-topic-publisher *grasp-status-topic*)
      (ros::advertise *grasp-status-topic* jsk_rviz_plugins::OverlayText 5)
      (unix:sleep 1))
    (ros::publish *grasp-status-topic*
                  (instance jsk_rviz_plugins::OverlayText :init
                            :width 1920 :height 400
                            :left 0 :top 0
                            :line_width 10 :text_size 50
                            :fg_color color
                            :text fmt))))

(defun publish-info (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-info s)
    (publish-status 'info s)))
(defun publish-warn (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-warn s)
    (publish-status 'warn s)))
(defun publish-error (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-error s)
    (publish-status 'error s)))

(defun vector3-> (applier v)
  (funcall applier
         (* (send v :x) 1000.0)
         (* (send v :y) 1000.0)
         (* (send v :z) 1000.0)))

(defun bounding-box->cube (msg)
  (let ((cds (ros::tf-pose->coords (send msg :pose)))
        (d (vector3-> 'list (send msg :dimensions)))
        (base->parent (send *tfl* :lookup-transform
                            *base-frame-id* (send msg :header :frame_id)
                            (ros::time 0))))
    (send *tfb* :send-transform
          (send (send base->parent :copy-worldcoords)
                :transform cds)
          *base-frame-id* "target_object" (send msg :header :stamp))
    (setq *box* (send (apply #'make-cube d)
          :transform
          (send (send base->parent :copy-worldcoords)
                :transform cds))
                )
    (send *box-arrow* :newcoords (send *box* :copy-worldcoords))
    )
  )

(defun grip ()   
    (setq *grasp-state* "grasp-closing")
    (publish-grasp-state)
    (publish-info "grasping...")
    (send *ri* :start-grasp *rarm*)
    (send *ri* :wait-interpolation)
    (unix:sleep 5)
    (let ((grasp-result (send *ri* :start-grasp *rarm* )))
      (send *ri* :wait-interpolation)
      (print grasp-result)
      ;;(unless (< 1 grasp-result)
      ;;  (publish-error "failed to grasp ~A" grasp-result)
      ;;  (send *ri* :stop-grasp *rarm* :wait t)
      ;;  (publish-info "back to pre grasp...")
      ;;  (return-from reaching-grip nil))
      )
)

(defun one-grip ()   
    (setq *grasp-state* "grasp-closing")
    (publish-grasp-state)
    (publish-info "grasping...")
    (send *ri* :start-grasp *rarm*)
    (send *ri* :wait-interpolation)
    (unix:sleep 5)
    (let ((grasp-result (send *ri* :start-grasp *rarm* )))
      (send *ri* :wait-interpolation)
      (print grasp-result)
      ;;(unless (< 1 grasp-result)
      ;;  (publish-error "failed to grasp ~A" grasp-result)
      ;;  (send *ri* :stop-grasp *rarm* :wait t)
      ;;  (publish-info "back to pre grasp...")
      ;;  (return-from reaching-grip nil))
      )
)

(defun pick ()
    (setq *grasp-state* "picking")
    (publish-grasp-state)
    (publish-info "picking up...")
    (send *baxter* *rarm* :move-end-pos #f(0 0 250) :world)
    (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (setq *finish-grasped* 1)
;;)

;;(defun postgrasp (&rest args)    
    (setq *grasp-state* "placing")
    (publish-grasp-state)
    (publish-info "placing ...")
    (send *baxter* *rarm* :move-end-pos #f(0 0 -230) :world)
    (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (print "save judge")
    ;; open glipper
    (send *ri* :stop-grasp *rarm* :wait t)
    (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (publish-info "click tabletop object!")
  )

(defun grasp-target-callback (msg)
  ;;(setq msg (one-shot-subscribe *grasp-target-bounding-box-topic*
  ;;                jsk_recognition_msgs::BoundingBoxArray
  ;;                :timeout 1000000))
  (print "clicked")
  ;;(setq *target-obj* (bounding-box->cube (elt (send msg :boxes) 0)))
  (setq *target-obj* (bounding-box->cube msg))

    (unless *target-obj*
      (publish-info "target point is nil")
      (return-from grasp-target-callback nil))
    (if *grasping-object-p*
        (progn
          (publish-warn "cancel grasping...")
          (send *ri* :stop-motion)
          (send *ri* :stop-grasp *rarm* :wait t)
          (setq *grasping-object-p* nil))
      (setq *grasping-object-p* t))
    (unless
	(progn
	  (send *baxter* *rarm* :inverse-kinematics
		(make-coords :pos (v+ (send *target-obj* :worldpos)
				       ;;(float-vector 0 (- (/ (car *d_xyz*) -2) 10) 30)
                                      (float-vector 0 -200 200))
			     ;;:rpy #f(1.54 1.37 -1.54))
			     :rpy #f(1.54 1.54 1.54))
		:rotation-axis :y :rotation-axis :z :translation-axis t))
      (publish-error "out of reach...")
      (send *pregrasp-point* :newcoords
		(make-coords :pos (v+ (send *target-obj* :worldpos)
				       ;;(float-vector 0 (- (/ (car *d_xyz*) -2) 10) 30)
                                      (float-vector 0 -200 200))
			     ;;:rpy #f(1.54 1.37 -1.54))
			     :rpy #f(1.54  1.54 1.54)))
      (send *co-ik-target* :newcoords (send *target-obj* :copy-worldcoords))
      (objects (list *baxter* *box* *co-ik-target* *pregrasp-point* *box-arrow*))
      (return-from grasp-target-callback nil))

    (publish-info "openning gripper...")
  
    (setq *grasp-state* "gripper-opening")
    (publish-grasp-state)
    (send *ri* :stop-grasp *rarm* :wait t)
    (send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (setq *grasp-state* "grasp-approaching")
    (publish-grasp-state)
    (publish-info "reaching...")
    (send *baxter* *rarm* :move-end-pos #f(0 120 -100) :world)
    (send *ri* :angle-vector (send *baxter* :angle-vector) 2000)
    (send *ri* :wait-interpolation) 
    (grip)
    (pick)
    (baxter-pregrasp-pose)
    (publish-info "click tabletop object!")
)

(defun wait-for-grasp-target ()
  (ros::subscribe *grasp-target-bounding-box-topic*
                  jsk_recognition_msgs::BoundingBox #'grasp-target-callback)
  (publish-info "click tabletop object!"))

(defun publish-grasp-state ()
  (ros::publish "grasp_state" (instance std_msgs::String :init :data *grasp-state*))) ;change from "/grasp_state" to "grasp_state"

(defun demo ()
  (eus_vive::baxter-init)
  ;;(softhand-init)
  (setq *grasping-object-p* nil)
  (setq *rarm* :rarm)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (ros::advertise "/grasp_state" std_msgs::String 1)
  (setq *grasp-state* "waiting")
  (baxter-pregrasp-pose)
  (wait-for-grasp-target)
  (ros::spin))

(demo)
