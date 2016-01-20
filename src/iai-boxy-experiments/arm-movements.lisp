;;; Copyright (c) 2016, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :iai-boxy-experiments)

(defparameter *left-arm* nil)

(defun bringup-scripting-env ()
  (start-ros-node "cram")
  (setf *left-arm* (make-beasty-handle "left_arm" 1 1337))
  (roslisp-beasty::beasty-init *left-arm* (beasty-init-gaol :left-arm nil)))

(defun get-arm-handle (arm)
  (declare (type keyword arm))
  (ecase arm
    (:left-arm *left-arm*)))

(defun beasty-base-config (arm)
  (declare (type keyword arm))
  (base-config :base-acceleration (beasty-base-acceleration arm)))

(defun beasty-base-acceleration (arm)
  (declare (type keyword arm))
  (ecase arm
    (:left-arm (make-instance 'wrench :translation (make-3d-vector 7.358 4.248 4.905)))))

(defun beasty-nullspace-config (arm)
  (declare (type keyword arm))
  (ecase arm
    (:left-arm (deactivated-nullspace-config))))

(defun deactivated-nullspace-config ()
  (nullspace-config
   :nullspace-stiffness 0.0
   :nullspace-damping 0.0
   :nullspace-dir (make-identity-vector)))

(defun deactivated-safety-settings ()
  (safety-settings
   :safety-strategies
   '(:CONTACT :IGNORE
     :LIGHT-COLLISION :IGNORE
     :STRONG-COLLISION :IGNORE
     :SEVERE-COLLISION :IGNORE)))

(defun beasty-tool-config (arm)
  (declare (type keyword arm))
  (ecase arm
    (:left-arm
     (tool-config :tool-mass 2.5 :tool-com (make-3d-vector 0.0 0.0 0.19)))))

(defun beasty-ee-config (arm)
  (declare (type keyword arm))
  (ecase arm
    (:left-arm
     (ee-config :ee-transform (make-instance 'transform :translation (make-3d-vector 0 0 0.319))))))

(defun beasty-default-goal (arm sim-p)
  (declare (type keyword arm))
  (goal
   :simulated-config (simulated-config :simulated-robot sim-p)
   :base-config (beasty-base-config arm)
   :ee-config (beasty-ee-config arm)
   :nullspace-config (beasty-nullspace-config arm)
   :tool-config (beasty-tool-config arm)
   :safety-settings (deactivated-safety-settings)
   :motor-power-config (motor-power-config :motor-power t)))

(defun beasty-init-gaol (arm sim-p)
  (declare (type keyword arm))
  (apply
   #'replace-plist
   (beasty-default-goal arm sim-p)
   (motor-power-config :motor-power nil)))
                          
(defun beasty-zero-g-goal (arm sim-p)
  (declare (type keyword arm))
  (replace-plist
   (beasty-default-goal arm sim-p)
   :command-type :gravity-compensation))

(defun beasty-joint-goal (goal-config arm sim-p)
  (declare (type keyword arm)
           (type list goal-config))
  (replace-plist
   (beasty-default-goal arm sim-p)
   :command-type :joint-impedance
   :joint-goal-config goal-config))

(defun beasty-cartesian-goal (goal-pose arm sim-p)
  (declare (type keyword arm)
           (type transform goal-pose))
  (replace-plist
   (beasty-default-goal arm sim-p)
   :command-type :cartesian-impedance
   :cartesian-goal-pose goal-pose))
   
;;;
;;; SCRIPTS
;;;

(defun goto-config (goal-config arm sim-p)
  (move-beasty-and-wait
   (get-arm-handle arm)
   (beasty-joint-goal goal-config arm sim-p)))

(defun goto-zero (arm sim-p)
  (goto-config (list 0 0 0 0 0 0 0) arm sim-p))

(defun goto-pose (goal-pose arm sim-p)
  (move-beasty-and-wait
   (get-arm-handle arm)
   (beasty-cartesian-goal goal-pose arm sim-p)))

(defun zero-g (arm sim-p)
  (ecase arm
    (:left-arm
     (beasty-switch-behavior *left-arm* (beasty-zero-g-goal arm sim-p)))))

(defun start-configuration (selector arm)
  (ecase arm
    (:left-arm (left-arm-start-configuration selector))))

(defun left-arm-start-configuration (selector)
  (ecase selector
    (:experiment1
     (list -1.1522574424743652
           0.9588661789894104
           -0.4551224410533905
           -1.4930555820465088
           -1.9568004608154297
           -1.6365865468978882
           -2.268911838531494))
    (:experiment2
     (list -1.1522574424743652
           0.9588661789894104
           -0.4551224410533905
           -1.4930555820465088
           -1.9568004608154297
           -1.6365865468978882
           -0.698911839))
    (:experiment3
     (list -0.39946720004081726
           1.4901506900787354
           -0.6373277306556702
           -1.6821023225784302
           0.5662044286727905
           -1.2445718050003052
           -0.03993133828043938))
    (:experiment4
     (list -0.9895088076591492
           0.7063331007957458
           -0.47424113750457764
           -1.8978658199310303
           -1.7367249011993408
           -0.769806444644928
           -1.4906091690063477))))

(defun left-arm-poses (selector)
  (ecase selector
    (:experiment1
     (let ((ee-transform
             ;; TODO: replace this with a call to TF!
             ;; calib_left_arm_base_link -> left_gripper_tool_frame
             (make-transform
              (make-3d-vector -0.310 0.437 0.102)
              (make-quaternion 0.868 0.093 -0.444 0.200))))
       (list
        (transform*
         ee-transform
         (make-instance
          'transform
          :translation (make-3d-vector 0 0 0.25)))
        (transform*
         ee-transform
         (make-instance
          'transform
          :translation (make-3d-vector -0.05 -0.05 0.25)))
        ee-transform
        )))
    (:experiment2
     (let ((ee-transform
             ;; TODO: replace this with a call to TF!
             ;; calib_left_arm_base_link -> left_gripper_tool_frame
             (make-transform
              (make-3d-vector -0.310 0.437 0.102)
              (make-quaternion 0.683 -0.539 -0.176 0.460))))
       (list
        (transform*
         ee-transform
         (make-instance
          'transform
          :translation (make-3d-vector 0 0 0.13)))
        ee-transform
        )
       
       ))
     

    ))

(defun run-experiment (selector arm sim-p)
  (goto-config (start-configuration selector arm) arm sim-p)
   (loop for goal-pose in (left-arm-poses selector) do
     (goto-pose goal-pose arm sim-p))
  )

;;;
;;; LISP UTILS
;;;

(defun plist-has (plist indicator)
  (declare (type list plist)
           (type keyword indicator))
  (find indicator plist))

(defun add-plist-once (plist indicator value)
  (declare (type keyword indicator)
           (type list plist))
  (if (not (plist-has plist indicator))
      (concatenate 'list (list indicator value) plist)
      plist))

(defun remove-plist-once (plist indicator)
  (declare (type keyword indicator)
           (type list plist))
  (if (plist-has plist indicator)
      (remove indicator (remove (getf plist indicator) plist :count 1) :count 1)
      plist))

(defun replace-plist-once (plist indicator value)
  (declare (type keyword indicator)
           (type list plist))
  (add-plist-once (remove-plist-once plist indicator) indicator value))

(defun add-plist (plist &rest indicator-value-pairs)
  (declare (type list plist indicator-value-pairs))
  (assert (evenp (length indicator-value-pairs)))
  (if indicator-value-pairs
      (apply #'add-plist (add-plist-once plist (first indicator-value-pairs) (second indicator-value-pairs)) (cddr indicator-value-pairs))
      plist))
  
(defun remove-plist (plist &rest indicators)
  (declare (type list plist indicators))
  (if indicators
      (apply #'remove-plist (remove-plist-once plist (first indicators)) (rest indicators))
      plist))

(defun replace-plist (plist &rest indicator-value-pairs)
  (declare (type list plist indicator-value-pairs))
  (assert (evenp (length indicator-value-pairs)))
  (if indicator-value-pairs
      (apply #'replace-plist (replace-plist-once plist (first indicator-value-pairs) (second indicator-value-pairs)) (cddr indicator-value-pairs))
      plist))
