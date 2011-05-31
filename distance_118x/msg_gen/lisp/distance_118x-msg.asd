
(cl:in-package :asdf)

(defsystem "distance_118x-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DistMsg" :depends-on ("_package_DistMsg"))
    (:file "_package_DistMsg" :depends-on ("_package"))
  ))