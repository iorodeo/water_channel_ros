
(cl:in-package :asdf)

(defsystem "controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ControllerCmd" :depends-on ("_package_ControllerCmd"))
    (:file "_package_ControllerCmd" :depends-on ("_package"))
  ))