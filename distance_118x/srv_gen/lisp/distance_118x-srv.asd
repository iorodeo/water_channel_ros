
(cl:in-package :asdf)

(defsystem "distance_118x-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DistSensorCtl" :depends-on ("_package_DistSensorCtl"))
    (:file "_package_DistSensorCtl" :depends-on ("_package"))
  ))